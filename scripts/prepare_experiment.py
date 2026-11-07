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
    ap.add_argument('--build',action='store_true',help='Build after preparing; source ESP-IDF export.sh first')
    args=ap.parse_args()
    if args.profile is None:
        args.profile='off' if args.stage in ('takeoff-land','rectangle') else '8x8_15'
    without_tof=args.profile=='off'
    if without_tof and args.stage in ('manual','record'):
        ap.error('--profile off requires --stage takeoff-land or rectangle')
    side,hz=(8,15) if without_tof else (int(args.profile[0]),int(args.profile.split('_')[1]))
    directory=ROOT/'build-experiments'/f'{args.stage}-{args.profile}'
    directory.mkdir(parents=True,exist_ok=True)
    config=directory/'sdkconfig'
    # Start with the local working hardware/network settings; never print credentials.
    text=(ROOT/'sdkconfig').read_text()
    settings={
        'CONFIG_NANOLITE_BENCH_MODE':'y' if args.stage=='manual' else 'n',
        'CONFIG_EXPERIMENT_MISSION_ENABLED':'n' if args.stage=='manual' else 'y',
        'CONFIG_EXPERIMENT_WITHOUT_TOF':'y' if without_tof else 'n',
        'CONFIG_NANOLITE_ENABLED':'n','CONFIG_APRILTAG_CAMERA_ENABLED':'n',
        'CONFIG_TOF_GRID_SIZE':str(side),'CONFIG_TOF_RATE_HZ':str(hz),
        'CONFIG_TOF_LEGACY_DEBUG':'n','CONFIG_DATA_TELEM_HZ':'100',
        'CONFIG_NANOLITE_POSE_RATE_HZ':'100',
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
    print(f'Prepared {args.stage}, {label}: {config}')
    if args.build:
        return subprocess.call([str(ROOT/'scripts/idf_logged.sh'),'-B',str(directory),'-D',f'SDKCONFIG={config}','build'],cwd=ROOT)
    return 0
if __name__=='__main__': sys.exit(main())
