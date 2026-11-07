#!/usr/bin/env python3
"""One receiver records telemetry and native ToF; autonomous flight runs onboard.

Manual recording: python3 laptop/run_experiment.py --mode manual --record-tof
Preview mission:  python3 laptop/run_experiment.py --mode rectangle
Execute mission:  python3 laptop/run_experiment.py --mode rectangle --execute
"""
from __future__ import annotations
import argparse
import base64
import csv
from dataclasses import asdict, fields
from datetime import datetime
import json
import math
from pathlib import Path
import secrets
import selectors
import socket
import time
from contextlib import ExitStack
from protocol import TelemetryPacket, parse_telemetry, CommandPacket, build_command, CMD_LAND
from experiment_protocol import decode_native, build_experiment, PHASES

TELEM_FIELDS=['host_receive_ns','host_monotonic_ns','source_ip']+[f.name for f in fields(TelemetryPacket)]
TOF_FIELDS=['host_receive_ns','host_monotonic_ns','source_ip','drone_id','boot_id','sensor','side','hz',
    'sequence','streamcount','queue_drops','flags','frame_reference_esp_us','reference_method',
    'readiness_halfwidth_us','frame_reference_px4_us','ready_before_us','ready_after_us',
    'read_start_us','read_end_us','previous_not_ready_us','send_us','clock_offset_us',
    'sync_locked','sync_rtt_us','sync_age_us','sync_samples','transfer_bytes']+[
    f'distance_mm_{i:02}' for i in range(64)]+[f'status_{i:02}' for i in range(64)]

def healthy(packet):
    return (packet.sync_locked==1 and packet.sync_rtt_us is not None and packet.sync_rtt_us<=5000
        and packet.sync_age_us<=2_000_000 and packet.px4_link_ok
        and packet.px4_pos_age_ms<250 and packet.px4_att_age_ms<250
        and all(v is not None and math.isfinite(v) for v in
                (packet.ned_x,packet.ned_y,packet.ned_z,packet.ned_vx,packet.ned_vy,packet.ned_vz,packet.heading_rad)))

def frame_row(frame, received, monotonic, source):
    row={k:v for k,v in asdict(frame).items() if k not in ('distances','statuses')}
    row.update(host_receive_ns=received,host_monotonic_ns=monotonic,source_ip=source,
        frame_reference_esp_us=frame.frame_reference_us,reference_method=frame.reference_method,
        readiness_halfwidth_us=frame.readiness_halfwidth_us,sync_locked=int(frame.sync_locked),
        frame_reference_px4_us=frame.frame_reference_us+frame.clock_offset_us if frame.sync_locked else '')
    row.update({f'distance_mm_{i:02}':v for i,v in enumerate(frame.distances)})
    row.update({f'status_{i:02}':v for i,v in enumerate(frame.statuses)})
    return row

def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=['manual','takeoff-land','rectangle'],default='manual')
    ap.add_argument('--execute',action='store_true',help='Send the selected autonomous mission command')
    ap.add_argument('--profile',choices=['off','8x8_15','4x4_30','4x4_60'],default='8x8_15',help='Expected sensor profile (off for baseline flight); rejects mismatched firmware')
    ap.add_argument('--record-tof',action='store_true',help='Save every received native ToF frame')
    ap.add_argument('--width',type=float,default=1.)
    ap.add_argument('--length',type=float,default=1.)
    ap.add_argument('--altitude',type=float,default=.5)
    ap.add_argument('--speed',type=float,default=.3)
    ap.add_argument('--drone-id',type=int,default=0)
    ap.add_argument('--source',help='Optional ESP32 IPv4 filter; otherwise learn from telemetry')
    ap.add_argument('--duration',type=float,help='Manual recording seconds; otherwise until Ctrl+C')
    ap.add_argument('--timeout',type=float,default=180.,help='Autonomous mission timeout in seconds')
    ap.add_argument('--preflight-timeout',type=float,default=30.)
    ap.add_argument('--telem-port',type=int,default=5005)
    ap.add_argument('--tof-port',type=int,default=5007)
    ap.add_argument('--cmd-port',type=int,default=5006)
    ap.add_argument('--output',type=Path,help='New run directory; must not already exist')
    args=ap.parse_args(argv)
    if args.duration is not None and (args.duration<=0 or args.mode!='manual'):
        ap.error('--duration must be positive and is only for manual recording')
    if args.timeout<=0 or args.preflight_timeout<=0: ap.error('timeouts must be positive')
    if args.profile=='off' and args.record_tof:
        ap.error('--record-tof requires an active ToF profile')
    expected_side,expected_hz=(0,0) if args.profile=='off' else (int(args.profile[0]),int(args.profile.split("_")[1]))
    mission_id=secrets.randbelow(2**32-1)+1
    try: command=build_experiment(mission_id,args.mode if args.mode!='manual' else 'rectangle',
                args.width,args.length,args.altitude,args.speed)
    except ValueError as exc: ap.error(str(exc))
    if args.mode!='manual' and not args.execute:
        path=f'width={args.width} m east, length={args.length} m north; ' if args.mode=='rectangle' else ''
        sequence=('3 s stationary at each corner; return to takeoff (0,0), then LAND.'
                  if args.mode=='rectangle' else 'settle, hold 3 s, then LAND.')
        sensors='; no ToF obstacle checks.' if args.profile=='off' else '.'
        print(f'Preview: {args.mode}; {path}'
              f'altitude={args.altitude} m above takeoff; setpoint speed={args.speed} m/s; '
              f'{sequence}\nToF: {args.profile}{sensors}\n'
              'Add --execute when ready to run this mission.')
        return 0
    if args.mode=='manual' and args.execute: ap.error('manual mode only records; omit --execute')
    directory=args.output or Path('results/experiments')/datetime.now().astimezone().strftime('%Y%m%d-%H%M%S-%f')
    directory.mkdir(parents=True,exist_ok=False)
    metadata=vars(args).copy(); metadata['output']=str(directory)
    metadata.update(mission_id=mission_id,started_unix_ns=time.time_ns(),schema_version=1,
        coordinates='north/east relative to captured takeoff; altitude above takeoff; fixed initial yaw',
        clock='PX4 boot us = ESP monotonic us + clock_offset_us',
        tof_enabled=args.profile!='off',
        tof_ranging_mode='disabled' if args.profile=='off' else 'autonomous',
        tof_integration_ms=None if args.profile=='off' else 5,
        tof_timestamp='Estimated data-ready time if bracketed, otherwise read end; optical exposure time is not known')
    (directory/'run.json').write_text(json.dumps(metadata,indent=2)+'\n')
    stats={'telemetry_packets':0,'tof_packets':0,'malformed_packets':0,'unsynced_tof':0,
           'sensors':{},'outcome':'recording','mission_sent':False}
    source=args.source; boot=None; began=time.monotonic(); accepted=False; terminal=False
    send_time=None; last_send=0; last_phase=None; last_print=0
    print(f'Recording to {directory}. Manual mode sends no commands.' if args.mode=='manual' else f'Recording to {directory}; waiting for synchronized PX4 telemetry before starting.')
    try:
        with ExitStack() as stack:
            selector=stack.enter_context(selectors.DefaultSelector())
            sockets={}
            for port,kind in ((args.telem_port,'telemetry'),(args.tof_port,'tof')):
                sock=stack.enter_context(socket.socket(socket.AF_INET,socket.SOCK_DGRAM))
                sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,2*1024*1024)
                sock.bind(('0.0.0.0',port)); sock.setblocking(False)
                selector.register(sock,selectors.EVENT_READ,kind); sockets[kind]=sock
            tx=stack.enter_context(socket.socket(socket.AF_INET,socket.SOCK_DGRAM))
            raw=stack.enter_context((directory/'packets.jsonl').open('x'))
            events=stack.enter_context((directory/'events.jsonl').open('x'))
            telem_handle=stack.enter_context((directory/'telemetry.csv').open('x',newline=''))
            tw=csv.DictWriter(telem_handle,fieldnames=TELEM_FIELDS); tw.writeheader()
            if args.record_tof:
                fw=csv.DictWriter(stack.enter_context((directory/'tof.csv').open('x',newline='')),
                                  fieldnames=TOF_FIELDS,extrasaction='ignore'); fw.writeheader()
            def event(name,**values):
                events.write(json.dumps(dict(event=name,host_unix_ns=time.time_ns(),**values))+'\n'); events.flush()
            def land_on_abort():
                if stats['mission_sent'] and not terminal and source:
                    for _ in range(3):
                        tx.sendto(build_command(CommandPacket(CMD_LAND)),(source,args.cmd_port))
                    event('host_requested_land')
            try:
                while True:
                    now=time.monotonic()
                    if args.mode=='manual' and args.duration and now-began>=args.duration:
                        stats['outcome']='completed'; break
                    if args.mode!='manual':
                        if send_time is None and now-began>args.preflight_timeout:
                            raise RuntimeError('Preflight timeout: require correct drone ID, sync lock, fresh PX4 data, disarmed flight image')
                        if send_time is not None and now-send_time>args.timeout:
                            raise RuntimeError('Mission timeout; requesting land')
                    for key,_ in selector.select(.05):
                        # Drain bursts promptly; disk writes are buffered, not fsync per packet.
                        for _ in range(128):
                            try: data,address=key.fileobj.recvfrom(65535)
                            except BlockingIOError: break
                            received=time.time_ns(); mono=time.monotonic_ns(); now=time.monotonic()
                            ip=address[0]
                            if source and source!=ip: continue
                            packet=parse_telemetry(data) if key.data=='telemetry' else None
                            frame=decode_native(data) if key.data=='tof' else None
                            if packet is None and frame is None:
                                if data[:1]!=b'\x03': stats['malformed_packets']+=1
                                continue
                            obj=packet or frame
                            if obj.drone_id!=args.drone_id: continue
                            if packet is not None and args.profile=='off' and (packet.tof_side,packet.tof_hz)!=(0,0):
                                raise RuntimeError('Baseline flight requires firmware reporting ToF disabled (0/0)')
                            if packet is not None and packet.tof_side is not None and (packet.tof_side,packet.tof_hz)!=(expected_side,expected_hz):
                                raise RuntimeError('Firmware ToF profile differs from --profile')
                            if frame is not None and (frame.side,frame.hz)!=(expected_side,expected_hz):
                                raise RuntimeError('Native frame profile differs from --profile')
                            if source is None:
                                if packet is None: continue
                                source=ip
                            current_boot=obj.boot_id
                            if boot is not None and current_boot is not None and current_boot!=boot:
                                event('esp_reboot',previous_boot=boot,boot_id=current_boot)
                                if stats['mission_sent']: raise RuntimeError('ESP reboot during mission')
                            if current_boot is not None: boot=current_boot
                            if packet is not None or args.record_tof:
                                raw.write(json.dumps(dict(kind=key.data,host_receive_ns=received,
                                    host_monotonic_ns=mono,source_ip=ip,payload_base64=base64.b64encode(data).decode()))+'\n')
                            if packet is not None:
                                stats['telemetry_packets']+=1
                                row=asdict(packet); row['vfh_blocked']=json.dumps(row['vfh_blocked'])
                                tw.writerow(dict(row,host_receive_ns=received,host_monotonic_ns=mono,source_ip=ip))
                                phase=packet.experiment_phase
                                phase_key=(packet.mission_id,phase,packet.corner)
                                if phase_key!=last_phase:
                                    last_phase=phase_key
                                    label=PHASES[phase] if phase is not None and phase<len(PHASES) else str(phase)
                                    event('phase',mission_id=packet.mission_id,phase=label,corner=packet.corner)
                                    print(f'phase={label} corner={packet.corner} sync={packet.sync_locked} rtt={packet.sync_rtt_us} us')
                                if args.mode!='manual':
                                    if send_time is None and healthy(packet) and packet.px4_armed==0 and packet.passive==0:
                                        tx.sendto(command,(source,args.cmd_port)); stats['mission_sent']=True
                                        send_time=last_send=now; event('mission_requested',mission_id=mission_id)
                                    elif send_time is not None and not accepted and now-last_send>=1:
                                        tx.sendto(command,(source,args.cmd_port)); last_send=now
                                    if packet.mission_id==mission_id:
                                        accepted=True
                                        if phase in (8,9,10,11):
                                            stats['outcome']=PHASES[phase]; terminal=True; break
                                if now-last_print>5:
                                    print(f'PX4 packets={stats["telemetry_packets"]} ToF packets={stats["tof_packets"]} '
                                          f'sync={packet.sync_locked} rtt={packet.sync_rtt_us} us; sample age={packet.px4_pos_age_ms} ms')
                                    last_print=now; raw.flush(); telem_handle.flush()
                            else:
                                stats['tof_packets']+=1
                                if not frame.sync_locked: stats['unsynced_tof']+=1
                                name=f'{frame.boot_id}:{frame.sensor}'
                                sensor=stats['sensors'].setdefault(name,dict(received=0,sequence_gaps=0,reordered=0,
                                    first_read_us=frame.read_end_us,last_read_us=frame.read_end_us,last_sequence=0,last_streamcount=None,
                                    streamcount_gaps=0,unbounded_timing=0,max_sync_rtt_us=0,
                                    minimum_wire_utilisation=frame.transfer_bytes*8*frame.hz*9/400000,
                                    maximum_queue_drops=0,max_read_duration_us=0,max_readiness_halfwidth_us=0,
                                    profile=f'{frame.side}x{frame.side}_{frame.hz}'))
                                if sensor['last_sequence']:
                                    delta=(frame.sequence-sensor['last_sequence'])&0xffffffff
                                    if delta==0 or delta>=0x80000000: sensor['reordered']+=1
                                    else: sensor['sequence_gaps']+=delta-1
                                if not sensor['last_sequence'] or 0<((frame.sequence-sensor['last_sequence'])&0xffffffff)<0x80000000:
                                    sensor['last_sequence']=frame.sequence
                                newest=not sensor['last_sequence'] or frame.sequence==sensor['last_sequence']
                                if newest and sensor['last_streamcount'] is not None:
                                    stream_delta=(frame.streamcount-sensor['last_streamcount'])%255
                                    sensor['streamcount_gaps']+=max(0,stream_delta-1)
                                if newest: sensor['last_streamcount']=frame.streamcount
                                sensor['unbounded_timing']+=int(frame.readiness_halfwidth_us is None)
                                if frame.sync_locked: sensor['max_sync_rtt_us']=max(sensor['max_sync_rtt_us'],frame.sync_rtt_us)
                                sensor['first_read_us']=min(sensor['first_read_us'],frame.read_end_us)
                                sensor['received']+=1; sensor['last_read_us']=max(sensor['last_read_us'],frame.read_end_us)
                                sensor['maximum_queue_drops']=max(sensor['maximum_queue_drops'],frame.queue_drops)
                                sensor['max_read_duration_us']=max(sensor['max_read_duration_us'],frame.read_end_us-frame.read_start_us)
                                sensor['max_readiness_halfwidth_us']=max(sensor['max_readiness_halfwidth_us'],frame.readiness_halfwidth_us or 0)
                                if args.record_tof: fw.writerow(frame_row(frame,received,mono,ip))
                        if terminal: break
                    if terminal: break
            except KeyboardInterrupt:
                stats['outcome']='interrupted'; land_on_abort()
            except Exception:
                land_on_abort(); raise
    except (OSError,RuntimeError) as exc:
        stats['outcome']='error'; stats['error']=str(exc); print(f'Error: {exc}')
    finally:
        for sensor in stats['sensors'].values():
            span=(sensor['last_read_us']-sensor['first_read_us'])/1e6
            sensor['observed_hz']=(sensor['received']-sensor['reordered']-1)/span if span>0 else None
        stats['finished_unix_ns']=time.time_ns()
        (directory/'summary.json').write_text(json.dumps(stats,indent=2)+'\n')
        print(f'Saved {directory}; outcome={stats["outcome"]}')
    return 0 if stats['outcome'] in ('completed','DONE','interrupted') else 1
if __name__=='__main__': raise SystemExit(main())
