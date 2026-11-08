import csv
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'laptop'))
from experiment_protocol import HEADER, decode_native, build_experiment
from protocol import parse_telemetry
from align_experiment import align, slerp

def telemetry(stamp=1010,boot=42,mode=0,phase=0,mission=0,side=8,hz=15):
    legacy=struct.pack('<BBfffBbf32sBH',1,0,1.,2.,.3,0,-1,0.,bytes(32),0,65535)
    old=legacy+struct.pack('<HHHBBB4f9fIIB',10,2,2,1,0,6,-.5,.1,.2,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,stamp,stamp,1)
    timing=struct.pack('<QqIIHBBIBBBBI',stamp*1000-10000,10000,2000,100000,8,1,phase,mission,0,side,hz,mode,boot)
    return old+timing

def native(side=8,flags=3,seq=1):
    n=side*side
    header=HEADER.pack(5,1,0,7,side,side,15 if side==8 else 60,flags,42,seq,0,
        999000,1000000,1000010,1001000,998000,1002000,10000,2000,100000,320,10,8)
    return header+struct.pack(f'<{n}h{n}B',*([-12]+[1200]*(n-1)),*([5]*n))

@pytest.mark.parametrize('side,size',[(4,136),(8,280)])
def test_native_schema_and_timing(side,size):
    data=native(side)
    assert len(data)==size
    frame=decode_native(data)
    assert frame.distances[0]==-12
    assert frame.side==side and frame.sensor==7 and frame.sync_locked
    assert frame.frame_reference_us==999000 and frame.readiness_halfwidth_us==1000
    assert frame.reference_method=='ready_bracket_midpoint'
    assert decode_native(data[:-1]) is None
    assert decode_native(data+b'x') is None
    assert decode_native(b'\x03'+data[1:]) is None

def test_unbounded_and_unsynced_frames():
    frame=decode_native(native(flags=4))
    assert not frame.sync_locked and frame.readiness_halfwidth_us is None
    assert frame.frame_reference_us==frame.read_end_us
    assert frame.reference_method=='read_end_unbounded'

def test_additive_telemetry_and_mission_command():
    data=telemetry()
    assert len(data)==165
    packet=parse_telemetry(data)
    assert packet.clock_offset_us==10000 and packet.boot_id==42
    assert packet.sync_locked==1 and packet.esp_send_us==1000000
    assert parse_telemetry(data[:125]).sync_locked is None
    assert parse_telemetry(data[:55]).q_w is None
    assert build_experiment(5,'rectangle-centred',4,5.5,.5,.3)[2]==3
    command=build_experiment(123,'rectangle',2,3,.5,.3)
    assert len(command)==23
    assert struct.unpack('<BBBI4f',command)==pytest.approx((2,7,2,123,2,3,.5,.3))
    for args in [('rectangle',float('nan'),1,.5,.3),('rectangle',1,1,.5,1)]:
        with pytest.raises(ValueError): build_experiment(123,*args)

def test_clock_and_rectangle_core(tmp_path):
    code=r'''
#include "clock_sync.h"
#include "experiment_plan.h"
#include <assert.h>
#include <math.h>
int main(void) {
 clock_sync_filter_t f; clock_sync_reset(&f);
 for(int i=0;i<8;i++) {
   long t=1000000+i*100000;
   assert(clock_sync_observe(&f,t,t+2000,t+1000+123456));
 }
 clock_sync_status_t s=clock_sync_status(&f,1702000);
 assert(s.locked && s.offset_us==123456 && s.rtt_us==2000);
 assert(!clock_sync_observe(&f,1800000,1830000,1923456));
 assert(!clock_sync_observe(&f,1800000,1799999,1923456));
 assert(!clock_sync_status(&f,4000000).locked);
 assert(clock_sync_observe(&f,4000000,4002000,3001000));
 assert(!clock_sync_status(&f,4002000).locked && f.count==1);
 for(int i=1;i<8;i++) {
   long t=4000000+i*100000;
   clock_sync_observe(&f,t,t+1000,t+500-1000000);
 }
 s=clock_sync_status(&f,4701000);
 assert(s.locked && s.offset_us==-1000000 && s.rtt_us==1000);
 experiment_xy_t c=experiment_corner(1,3,2); assert(c.x==3 && c.y==0);
 c=experiment_corner(2,3,2); assert(c.x==3 && c.y==2);
 c=experiment_corner(3,3,2); assert(c.x==0 && c.y==2);
 c=experiment_corner(4,3,2); assert(c.x==0 && c.y==0);
 /* rectangle from the middle of the bottom edge: W, N, E, S, back */
 assert(experiment_waypoint_count(3)==6 && experiment_waypoint_count(2)==5 && experiment_waypoint_count(1)==1);
 c=experiment_waypoint(3,1,5.5f,4); assert(c.x==0 && c.y==-2);
 c=experiment_waypoint(3,2,5.5f,4); assert(c.x==5.5f && c.y==-2);
 c=experiment_waypoint(3,3,5.5f,4); assert(c.x==5.5f && c.y==2);
 c=experiment_waypoint(3,4,5.5f,4); assert(c.x==0 && c.y==2);
 c=experiment_waypoint(3,5,5.5f,4); assert(c.x==0 && c.y==0);
 c=experiment_waypoint(2,2,3,2); assert(c.x==3 && c.y==2);
 c=experiment_step((experiment_xy_t){0,0},(experiment_xy_t){1,0},.3,.02);
 assert(fabsf(c.x-.006f)<1e-6 && c.y==0);
 c=experiment_step((experiment_xy_t){.999,0},(experiment_xy_t){1,0},.3,.02);
 assert(c.x==1);
 assert(!experiment_settled(.05,.2,.1));
 assert(experiment_settled(.05,.01,.1));
 return 0;
}'''
    source=tmp_path/'core.c'; source.write_text(code)
    binary=tmp_path/'core'
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'main'),str(source),
        str(ROOT/'main/clock_sync.c'),str(ROOT/'main/experiment_plan.c'),'-lm','-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)

def free_port():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1',0)); return sock.getsockname()[1]

def test_passive_receiver_and_offline_alignment(tmp_path):
    ports=[free_port(),free_port(),free_port()]
    while len(set(ports))!=3: ports=[free_port(),free_port(),free_port()]
    output=tmp_path/'run'
    process=subprocess.Popen([sys.executable,'-u',str(ROOT/'laptop/run_experiment.py'),
        '--mode','manual','--record-tof','--duration','0.8','--telem-port',str(ports[0]),
        '--tof-port',str(ports[1]),'--cmd-port',str(ports[2]),'--output',str(output)],
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        # The first print precedes bind; retry input during the short startup window.
        assert 'Recording to' in process.stdout.readline()
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as commands, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as tx:
            commands.bind(('127.0.0.1',ports[2])); commands.settimeout(.1)
            for i in range(3):
                tx.sendto(telemetry(1000,mode=1),('127.0.0.1',ports[0]))
                tx.sendto(telemetry(1020,mode=1),('127.0.0.1',ports[0]))
                tx.sendto(native(seq=i+1),('127.0.0.1',ports[1]))
                time.sleep(.03)
            stdout,stderr=process.communicate(timeout=5)
            assert process.returncode==0,(stdout,stderr)
            with pytest.raises(socket.timeout): commands.recvfrom(4096)
        summary=json.loads((output/'summary.json').read_text())
        assert summary['telemetry_packets']>=2 and summary['tof_packets']>=1
        count,valid=align(output)
        assert count==summary['tof_packets'] and valid==count
        aligned=list(csv.DictReader((output/'aligned.csv').open()))
        assert float(aligned[0]['aligned_ned_z'])==pytest.approx(-.5)
        assert float(aligned[0]['aligned_q_w'])==pytest.approx(1)
    finally:
        if process.poll() is None: process.kill(); process.wait()

def test_alignment_slerp_and_no_extrapolation():
    assert slerp([1,0,0,0],[-1,0,0,0],.5)==pytest.approx([1,0,0,0])
    assert slerp([1,0,0,0],[0,0,0,1],.5)==pytest.approx([math.sqrt(.5),0,0,math.sqrt(.5)])
    from align_experiment import bracket
    samples={'times':[100,200],'rows':[{},{}]}
    assert bracket(samples,50,1000) is None
    assert bracket(samples,250,1000) is None
    assert bracket(samples,150,50) is None

@pytest.mark.parametrize('profile,side,hz', [('8x8_15',8,15),('off',0,0)])
def test_autonomous_receiver_waits_for_sync_and_tracks_ack(tmp_path,profile,side,hz):
    ports=[free_port(),free_port(),free_port()]
    while len(set(ports))!=3: ports=[free_port(),free_port(),free_port()]
    output=tmp_path/'autonomous'
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as commands, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as tx:
        commands.bind(('127.0.0.1',ports[2])); commands.settimeout(.1)
        process=subprocess.Popen([sys.executable,'-u',str(ROOT/'laptop/run_experiment.py'),
            '--mode','rectangle','--execute','--timeout','3','--preflight-timeout','2',
            '--profile',profile,
            '--telem-port',str(ports[0]),'--tof-port',str(ports[1]),'--cmd-port',str(ports[2]),
            '--width','2','--length','3','--output',str(output)],
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            assert 'Recording to' in process.stdout.readline()
            bad=bytearray(telemetry(side=side,hz=hz)); bad[151]=0  # sync lock byte in additive extension
            for _ in range(3):
                tx.sendto(bad,('127.0.0.1',ports[0])); time.sleep(.02)
            with pytest.raises(socket.timeout): commands.recvfrom(4096)
            commands.settimeout(1)
            tx.sendto(telemetry(side=side,hz=hz),('127.0.0.1',ports[0]))
            data,_=commands.recvfrom(4096)
            kind,cmd,mode,mission,width,length,alt,speed=struct.unpack('<BBBI4f',data)
            assert (kind,cmd,mode,width,length)==(2,7,2,2,3)
            tx.sendto(telemetry(phase=1,mission=mission,side=side,hz=hz),('127.0.0.1',ports[0]))
            tx.sendto(telemetry(phase=8,mission=mission,side=side,hz=hz),('127.0.0.1',ports[0]))
            stdout,stderr=process.communicate(timeout=5)
            assert process.returncode==0,(stdout,stderr)
            summary=json.loads((output/'summary.json').read_text())
            assert summary['outcome']=='DONE' and summary['mission_sent']
            metadata=json.loads((output/'run.json').read_text())
            assert metadata['tof_enabled']==(profile!='off')
        finally:
            if process.poll() is None: process.kill(); process.wait()

@pytest.mark.parametrize('side,hz', [(8,15),(0,15),(8,0)])
def test_baseline_rejects_active_or_inconsistent_tof_profile(tmp_path,side,hz):
    ports=[free_port(),free_port(),free_port()]
    while len(set(ports))!=3: ports=[free_port(),free_port(),free_port()]
    output=tmp_path/'mismatch'
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as commands, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as tx:
        commands.bind(('127.0.0.1',ports[2])); commands.settimeout(.1)
        process=subprocess.Popen([sys.executable,'-u',str(ROOT/'laptop/run_experiment.py'),
            '--mode','takeoff-land','--profile','off','--execute','--preflight-timeout','1',
            '--telem-port',str(ports[0]),'--tof-port',str(ports[1]),'--cmd-port',str(ports[2]),
            '--output',str(output)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            assert 'Recording to' in process.stdout.readline()
            for _ in range(3):
                tx.sendto(telemetry(side=side,hz=hz),('127.0.0.1',ports[0])); time.sleep(.03)
            stdout,stderr=process.communicate(timeout=5)
            assert process.returncode!=0,(stdout,stderr)
            summary=json.loads((output/'summary.json').read_text())
            assert not summary['mission_sent']
            with pytest.raises(socket.timeout): commands.recvfrom(4096)
        finally:
            if process.poll() is None: process.kill(); process.wait()

@pytest.mark.parametrize('stage,profile,disabled', [
    ('takeoff-land',None,True),('rectangle','off',True),
    ('record','8x8_15',False),('record','4x4_30',False),('manual',None,False)])
def test_prepared_profiles_isolate_flight_and_sensor_workloads(tmp_path,stage,profile,disabled):
    # A minimal local config avoids copying credentials into test artifacts.
    script=tmp_path/'scripts/prepare_experiment.py'
    script.parent.mkdir(); script.write_text((ROOT/'scripts/prepare_experiment.py').read_text())
    original='CONFIG_NANOSLAM_ENABLED=y\nCONFIG_EXPERIMENT_WITHOUT_TOF=y\nCONFIG_TOF_RATE_HZ=60\n'
    (tmp_path/'sdkconfig').write_text(original)
    command=[sys.executable,str(script),'--stage',stage]
    if profile is not None: command+=['--profile',profile]
    subprocess.run(command,check=True,capture_output=True)
    selected=profile or ('off' if disabled else '8x8_15')
    config=tmp_path/f'build-experiments/{stage}-{selected}/sdkconfig'
    lines=config.read_text().splitlines()
    assert ('CONFIG_EXPERIMENT_WITHOUT_TOF=y' in lines)==disabled
    assert '# CONFIG_NANOSLAM_ENABLED is not set' in lines
    assert '# CONFIG_APRILTAG_CAMERA_ENABLED is not set' in lines
    if not disabled:
        assert f'CONFIG_TOF_GRID_SIZE={selected[0]}' in lines
        assert f'CONFIG_TOF_RATE_HZ={selected.split("_")[1]}' in lines
    assert config.stat().st_mode & 0o777 == 0o600
    assert (tmp_path/'sdkconfig').read_text()==original
