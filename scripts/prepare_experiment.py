#!/usr/bin/env python3
"""Create an isolated experiment build configuration. Never flashes hardware."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--stage',choices=['manual','takeoff-land','rectangle','record'],default='manual')
    ap.add_argument('--profile',choices=['off','8x8_15','4x4_30','4x4_60'],
                    help='Default: off for takeoff-land/rectangle, 8x8_15 for manual/record')
    ap.add_argument('--slam',action='store_true',
                    help='Run onboard NanoSLAM (ICP + PGO) and upload the graph to the laptop after landing; '
                         'needs a ToF profile (8x8_15, 4x4_30 or 4x4_60). Receive with laptop/nanoslam_receive.py')
    ap.add_argument('--slam-interval-ms',type=int,
                    help='NanoSLAM pose interval (default 1500). The graph holds 64 poses, so '
                         'flights longer than ~90 s need e.g. 2000')
    ap.add_argument('--tof-integration-ms',type=int,
                    help='VL53L5CX integration time (default 5; at most 1000/Hz - 6)')
    ap.add_argument('--name',help='Suffix for the build directory, keeping other builds intact')
    ap.add_argument('--build',action='store_true',help='Build after preparing; source ESP-IDF export.sh first')
    args=ap.parse_args()
    if args.profile is None:
        args.profile='off' if args.stage in ('takeoff-land','rectangle') else '8x8_15'
    without_tof=args.profile=='off'
    if without_tof and args.stage in ('manual','record'):
        ap.error('--profile off requires --stage takeoff-land or rectangle')
    if args.slam and args.profile=='off':
        ap.error('--slam needs a ToF profile (8x8_15, 4x4_30 or 4x4_60)')
    if args.tof_integration_ms is not None:
        if without_tof: ap.error('--tof-integration-ms needs a ToF profile')
        rate=int(args.profile.split('_')[1])
        if not 2<=args.tof_integration_ms<=1000//rate-6:
            ap.error(f'--tof-integration-ms must be 2-{1000//rate-6} at {rate} Hz')
    side,hz=(8,15) if without_tof else (int(args.profile[0]),int(args.profile.split('_')[1]))
    if args.slam_interval_ms is not None and (not args.slam or not 600<=args.slam_interval_ms<=10000):
        ap.error('--slam-interval-ms needs --slam and must be 600-10000')
    if args.name and not args.name.replace('-','').replace('_','').isalnum():
        ap.error('--name may only contain letters, digits, - and _')
    directory=ROOT/'build-experiments'/f'{args.stage}-{args.profile}{"-slam" if args.slam else ""}{"-"+args.name if args.name else ""}'
    directory.mkdir(parents=True,exist_ok=True)
    config=directory/'sdkconfig'
    # Start with the local working hardware/network settings; never print credentials.
    text=(ROOT/'sdkconfig').read_text()
    settings={
        'CONFIG_BENCH_MODE':'y' if args.stage=='manual' else 'n',
        'CONFIG_EXPERIMENT_MISSION_ENABLED':'n' if args.stage=='manual' else 'y',
        'CONFIG_EXPERIMENT_WITHOUT_TOF':'y' if without_tof else 'n',
        'CONFIG_NANOSLAM_ENABLED':'y' if args.slam else 'n','CONFIG_APRILTAG_CAMERA_ENABLED':'n',
        'CONFIG_TOF_GRID_SIZE':str(side),'CONFIG_TOF_RATE_HZ':str(hz),
        'CONFIG_TOF_LEGACY_DEBUG':'n','CONFIG_DATA_TELEM_HZ':'100',
        'CONFIG_PX4_POSE_RATE_HZ':'100',
        **({'CONFIG_NANOSLAM_RECORD_INTERVAL_MS':str(args.slam_interval_ms)} if args.slam_interval_ms else {}),
        **({'CONFIG_TOF_INTEGRATION_MS':str(args.tof_integration_ms)} if args.tof_integration_ms else {}),
        'CONFIG_VL53L5CX_NB_TARGET_PER_ZONE':'1',
        'CONFIG_VL53L5CX_DISABLE_DISTANCE_MM':'n','CONFIG_VL53L5CX_DISABLE_TARGET_STATUS':'n',
        'CONFIG_VL53L5CX_DISABLE_NB_TARGET_DETECTED':'n',
    }
    for field in ('AMBIENT_PER_SPAD','NB_SPADS_ENABLED','SIGNAL_PER_SPAD','RANGE_SIGMA_MM','REFLECTANCE_PERCENT','MOTION_INDICATOR'):
        settings['CONFIG_VL53L5CX_DISABLE_'+field]='y'
    lines=[line for line in text.splitlines() if not any(line.startswith(k+'=') or line==f'# {k} is not set' for k in settings)]
    lines += [f'# {k} is not set' if v=='n' else f'{k}={v}' for k,v in settings.items()]
    config.write_text('\n'.join(lines)+'\n'); os.chmod(config,0o600)
    label='ToF OFF (no obstacle checks)' if without_tof else f'{side}x{side} {hz} Hz'
    print(f'Prepared {args.stage}, {label}{", NanoSLAM on" if args.slam else ""}: {config}')
    if args.build:
        return subprocess.call([str(ROOT/'scripts/idf_logged.sh'),'-B',str(directory),'-D',f'SDKCONFIG={config}','build'],cwd=ROOT)
    return 0
if __name__=='__main__': sys.exit(main())
