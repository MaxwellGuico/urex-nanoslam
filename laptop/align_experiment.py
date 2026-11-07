#!/usr/bin/env python3
"""Align recorded native ToF frame references with bracketing PX4 pose samples.

Preserves tof.csv, writes aligned.csv without extrapolating, and flags uncertain
readiness timing. Position/velocity use linear interpolation; attitude uses
shortest-path quaternion SLERP. No optical exposure timestamp is inferred.
"""
from __future__ import annotations
import argparse
import bisect
import csv
import math
from pathlib import Path

POSITION=['ned_x','ned_y','ned_z','ned_vx','ned_vy','ned_vz']
QUATERNION=['q_w','q_x','q_y','q_z']

def bracket(samples,stamp,max_gap_us):
    index=bisect.bisect_left(samples['times'],stamp)
    if index<len(samples['times']) and samples['times'][index]==stamp:
        row=samples['rows'][index]; return row,row,0.,0
    if index==0 or index==len(samples['times']): return None
    a,b=samples['times'][index-1:index+1]
    if b-a>max_gap_us: return None
    return samples['rows'][index-1],samples['rows'][index],(stamp-a)/(b-a),b-a

def slerp(a,b,fraction):
    def normalize(q):
        length=math.sqrt(sum(x*x for x in q))
        if not math.isfinite(length) or length<1e-9: raise ValueError('invalid quaternion')
        return [x/length for x in q]
    a,b=normalize(a),normalize(b)
    dot=sum(x*y for x,y in zip(a,b))
    if dot<0: b=[-x for x in b]; dot=-dot
    dot=min(1.,max(-1.,dot))
    if dot>.9995: return normalize([(1-fraction)*x+fraction*y for x,y in zip(a,b)])
    angle=math.acos(dot); divisor=math.sin(angle)
    wa=math.sin((1-fraction)*angle)/divisor; wb=math.sin(fraction*angle)/divisor
    return [wa*x+wb*y for x,y in zip(a,b)]

def samples_from(rows,stamp_key,value_keys):
    unique={}
    for row in rows:
        if row.get(stamp_key) and row.get('sync_locked')=='1' and all(row.get(k) for k in value_keys):
            if int(row.get('px4_pos_age_ms') or 65535)>=250 or int(row.get('px4_att_age_ms') or 65535)>=250: continue
            values=[float(row[k]) for k in value_keys]
            if all(math.isfinite(v) for v in values): unique[int(row[stamp_key])*1000]=row
    times=sorted(unique)
    return dict(times=times,rows=[unique[t] for t in times])

def align(directory,max_gap_us=100000,max_halfwidth_us=10000,output_path=None,unbounded_period_us=None):
    # unbounded_period_us (opt-in): a frame read without an earlier not-ready poll became
    # ready at most one frame period before its read started. Re-reference it to the
    # middle of [read_start - period, read_start] with halfwidth period/2, instead of
    # rejecting it. Off by default; the method is written to reference_method.
    # Divide at ESP boot IDs and PX4 boot-time resets; never interpolate across resets.
    sessions={}
    by_boot={}
    with (directory/'telemetry.csv').open() as handle:
        for row in csv.DictReader(handle):
            if not row.get('boot_id') or not row.get('esp_send_us') or not row.get('px4_pos_boot_ms'): continue
            by_boot.setdefault(row['boot_id'],[]).append(row)
    for boot,rows in by_boot.items():
        groups=sessions.setdefault(boot,[])
        for row in sorted(rows,key=lambda r:int(r['esp_send_us'])):
            stamp=int(row['px4_pos_boot_ms'])
            if not groups or stamp+100<int(groups[-1][-1]['px4_pos_boot_ms']): groups.append([])
            groups[-1].append(row)
    tables={}
    for boot,groups in sessions.items():
        tables[boot]=[(int(group[0]['esp_send_us']) if i else 0,
                       samples_from(group,'px4_pos_boot_ms',POSITION),
                       samples_from(group,'px4_att_boot_ms',QUATERNION)) for i,group in enumerate(groups)]
    extras=['alignment_valid','alignment_notes','position_bracket_us','attitude_bracket_us']+['aligned_'+k for k in POSITION+QUATERNION]
    count=valid=0
    destination=directory/'aligned.csv' if output_path is None else Path(output_path)
    with (directory/'tof.csv').open() as source, destination.open('x',newline='') as target:
        reader=csv.DictReader(source)
        writer=csv.DictWriter(target,fieldnames=reader.fieldnames+extras); writer.writeheader()
        for row in reader:
            count+=1; notes=[]; row['alignment_valid']=0
            if (unbounded_period_us and row['reference_method']!='ready_bracket_midpoint'
                    and row.get('read_start_us') and row.get('frame_reference_px4_us')):
                shift=int(row['read_start_us'])-unbounded_period_us//2-int(row['frame_reference_esp_us'])
                row['frame_reference_esp_us']=int(row['frame_reference_esp_us'])+shift
                row['frame_reference_px4_us']=int(row['frame_reference_px4_us'])+shift
                row['readiness_halfwidth_us']=unbounded_period_us//2
                row['reference_method']='assumed_frame_period'
            if row['sync_locked']!='1' or not row['frame_reference_px4_us']:
                notes.append('clock_not_locked')
            else:
                groups=tables.get(row['boot_id'],[])
                index=bisect.bisect_right([g[0] for g in groups],int(row['frame_reference_esp_us']))-1
                if index<0: notes.append('no_telemetry_session')
                else:
                    stamp=int(row['frame_reference_px4_us']); _,pos,att=groups[index]
                    p=bracket(pos,stamp,max_gap_us); q=bracket(att,stamp,max_gap_us)
                    if p:
                        a,b,f,gap=p; row['position_bracket_us']=gap
                        for key in POSITION: row['aligned_'+key]=(1-f)*float(a[key])+f*float(b[key])
                    else: notes.append('no_nearby_position_bracket')
                    if q:
                        a,b,f,gap=q; row['attitude_bracket_us']=gap
                        try:
                            values=slerp([float(a[k]) for k in QUATERNION],[float(b[k]) for k in QUATERNION],f)
                            row.update(zip(['aligned_'+k for k in QUATERNION],values))
                        except ValueError: notes.append('invalid_quaternion')
                    else: notes.append('no_nearby_attitude_bracket')
            if row['reference_method'] not in ('ready_bracket_midpoint','assumed_frame_period'): notes.append('unbounded_readiness_time')
            elif int(row['readiness_halfwidth_us'])>max_halfwidth_us: notes.append('wide_readiness_bracket')
            row['alignment_valid']=int(not notes)
            row['alignment_notes']=';'.join(notes)
            valid+=int(not notes); writer.writerow(row)
    return count,valid

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run',type=Path)
    ap.add_argument('--max-pose-gap-ms',type=float,default=100)
    ap.add_argument('--max-readiness-halfwidth-ms',type=float,default=10)
    ap.add_argument('--assume-unbounded-period-ms',type=float,
                    help='Opt-in: bound unbounded frames by one frame period (e.g. 66.7 at 15 Hz)')
    args=ap.parse_args()
    if args.max_pose_gap_ms<=0 or args.max_readiness_halfwidth_ms<=0: ap.error('timing limits must be positive')
    period=int(args.assume_unbounded_period_ms*1000) if args.assume_unbounded_period_ms else None
    count,valid=align(args.run,int(args.max_pose_gap_ms*1000),int(args.max_readiness_halfwidth_ms*1000),
                      unbounded_period_us=period)
    print(f'Wrote {args.run/"aligned.csv"}: {valid}/{count} frames passed clock, readiness and pose-bracket checks.')
if __name__=='__main__': main()
