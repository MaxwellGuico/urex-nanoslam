#!/usr/bin/env python3
"""Compare ToF signal quality between recordings, e.g. integration-time tests.

For each results/experiments/<run> (recorded with --record-tof) it reports,
for the centre rows that NanoSLAM uses (rows 2-5 of 8x8, rows 1-2 of 4x4):

  valid      zones with target status 5 or 9
  no target  zones with status 255
  range      p50 / p95 / max of valid distances
  usable     columns per frame that pass the onboard scan rules (>= 2 valid
             rows within 0.3-2.5 m that agree within 100 mm), i.e. what SLAM gets
  per sensor valid fraction for each of the 8 sensors

The integration time comes from the tof_integration_ms column that firmware
since 2026-11-08 reports in telemetry.csv.

Usage:
  python3 laptop/tof_signal_report.py results/experiments/<run1> results/experiments/<run2> ...
  python3 laptop/tof_signal_report.py --plot results/previews/tof_signal.png <runs...>
"""
from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path

import numpy as np

MIN_MM, MAX_MM, MAX_SPREAD_MM = 300, 2500, 100   # main/.../scan.h


def integration_ms(run: Path):
    values = Counter()
    with (run / 'telemetry.csv').open() as handle:
        for row in csv.DictReader(handle):
            if row.get('tof_integration_ms') not in (None, ''):
                values[int(row['tof_integration_ms'])] += 1
    return values.most_common(1)[0][0] if values else None


def analyse(run: Path) -> dict:
    status, dist, sensors, usable = [], [], [], []
    side = None
    with (run / 'tof.csv').open() as handle:
        for row in csv.DictReader(handle):
            side = int(row['side'])
            rows = range(side // 4, 3 * side // 4)
            zones = [r * side + c for r in rows for c in range(side)]
            st = np.array([int(row[f'status_{z:02}']) for z in zones]).reshape(len(rows), side)
            ds = np.array([float(row[f'distance_mm_{z:02}']) for z in zones]).reshape(len(rows), side)
            status.append(st)
            dist.append(ds)
            sensors.append(int(row['sensor']))
            ok = np.isin(st, (5, 9)) & (ds >= MIN_MM) & (ds <= MAX_MM)
            n = 0
            for c in range(side):
                v = ds[ok[:, c], c]
                if len(v) >= 2 and v.max() - v.min() <= MAX_SPREAD_MM:
                    n += 1
            usable.append(n)
    if not status:
        raise ValueError(f'{run}: no ToF frames (record with --record-tof)')
    st, ds, sensors = np.array(status), np.array(dist), np.array(sensors)
    valid = np.isin(st, (5, 9))
    per_sensor = {s: float(valid[sensors == s].mean()) for s in sorted(set(sensors.tolist()))}
    return dict(run=run.name, side=side, integration=integration_ms(run), frames=len(st),
                valid=float(valid.mean()), no_target=float(np.mean(st == 255)),
                range=np.percentile(ds[valid], [50, 95]).tolist() + [float(ds[valid].max())] if valid.any() else [0, 0, 0],
                usable=float(np.mean(usable)), usable_max=side, per_sensor=per_sensor,
                valid_ranges=ds[valid])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', type=Path, nargs='+')
    ap.add_argument('--plot', type=Path, help='Write a range-histogram comparison PNG')
    args = ap.parse_args(argv)
    results = [analyse(run) for run in args.runs]

    print(f"{'run':36s} {'integr.':>7s} {'valid':>6s} {'no tgt':>6s} {'range p50/p95/max (m)':>22s} "
          f"{'usable cols/frame':>17s}  per-sensor valid")
    for r in results:
        integ = f"{r['integration']} ms" if r['integration'] is not None else '?'
        p50, p95, mx = (x / 1000 for x in r['range'])
        sens = ' '.join(f'{v * 100:.0f}' for v in r['per_sensor'].values())
        print(f"{r['run']:36s} {integ:>7s} {r['valid'] * 100:5.0f}% {r['no_target'] * 100:5.0f}% "
              f"{p50:7.2f} / {p95:4.2f} / {mx:4.2f}   {r['usable']:6.1f} of {r['usable_max']}      {sens}")

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 4.5))
        bins = np.arange(0, 4200, 100)
        for r in results:
            label = f"{r['integration']} ms" if r['integration'] is not None else r['run']
            ax.hist(r['valid_ranges'], bins=bins, histtype='step', lw=2,
                    weights=np.full(len(r['valid_ranges']), 1 / r['frames']),
                    label=f"{label}: {r['valid'] * 100:.0f}% valid")
        ax.axvline(MAX_MM, color='#999', ls='--', lw=1)
        ax.set_xlabel('valid target distance (mm)')
        ax.set_ylabel('valid centre-row zones per frame')
        ax.set_title('ToF returns by integration time (centre rows)')
        ax.legend()
        ax.grid(alpha=.3)
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=130)
        print(f'Plot: {args.plot}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
