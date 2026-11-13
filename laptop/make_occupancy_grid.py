#!/usr/bin/env python3
"""Replay offline_slam.py results into a Nav2 map.pgm + map.yaml.

Black=occupied, white=free, grey=unknown. Origin is the lower-left pixel's
corner in local ENU (East,North), in metres. No ROS installation is needed.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np


def transform(points, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def line_cells(start, end):
    """Integer Bresenham cells, inclusive of both sensor and obstacle cells."""
    x, y = map(int, start)
    tx, ty = map(int, end)
    dx, dy = abs(tx-x), -abs(ty-y)
    sx, sy = (1 if x < tx else -1), (1 if y < ty else -1)
    error = dx + dy
    while True:
        yield x, y
        if x == tx and y == ty:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x += sx
        if twice <= dx:
            error += dx
            y += sy


def _dilate(mask):
    """3x3 binary dilation."""
    padded = np.pad(mask, 1)
    out = np.zeros_like(mask)
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            out |= padded[dy:dy + mask.shape[0], dx:dx + mask.shape[1]]
    return out


def rasterize(points, origins, scan_ids, poses, resolution=.05, margin=.5,
              hit=.85, miss=-.4, max_cells=4_000_000, *,
              clear_points=None, clear_origins=None, clear_scan_ids=None, hits_only=False):
    """Log-odds grid from hit beams (points/origins, body frame per scan).

    Free-space rule D (default when clear beams are given): clear_* are beams
    that saw nothing within range (endpoint at the range limit, body frame).
    They mark free space along their length but stop before any wall found by
    the hit beams, so walls are never erased and nothing leaks through them.
    hits_only=True (rule A) ignores them."""
    points, origins = np.asarray(points), np.asarray(origins)
    scan_ids, poses = np.asarray(scan_ids), np.asarray(poses)
    use_clear = not hits_only and clear_points is not None and len(clear_points) > 0
    if use_clear:
        clear_points, clear_origins = np.asarray(clear_points), np.asarray(clear_origins)
        clear_scan_ids = np.asarray(clear_scan_ids)
        if (clear_points.ndim != 2 or clear_points.shape[1] != 2 or clear_origins.shape != clear_points.shape
                or clear_scan_ids.shape != (len(clear_points),)
                or not np.issubdtype(clear_scan_ids.dtype, np.integer)
                or clear_scan_ids.min() < 0 or clear_scan_ids.max() >= len(poses)
                or not (np.isfinite(clear_points).all() and np.isfinite(clear_origins).all())):
            raise ValueError('Invalid clear-beam arrays')
    if (points.ndim != 2 or points.shape[1] != 2 or origins.shape != points.shape
            or scan_ids.shape != (len(points),) or poses.ndim != 2 or poses.shape[1] != 3
            or len(points) == 0 or len(poses) == 0):
        raise ValueError('Invalid or empty SLAM result arrays')
    if not all(np.isfinite(x).all() for x in (points, origins, scan_ids, poses)):
        raise ValueError('Non-finite SLAM result')
    if not np.issubdtype(scan_ids.dtype, np.integer) or scan_ids.min() < 0 or scan_ids.max() >= len(poses):
        raise ValueError('Invalid scan IDs')
    # Vectorized local scan -> world projection; preserve sensor lever arms.
    def world(values, ids):
        p = poses[ids]
        c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
        return np.column_stack([c*values[:, 0]-s*values[:, 1]+p[:, 0],
                                s*values[:, 0]+c*values[:, 1]+p[:, 1]])
    endpoints, starts = world(points, scan_ids), world(origins, scan_ids)
    extent = [endpoints, starts, poses[:, :2]]
    if use_clear:
        clear_ends = world(clear_points, clear_scan_ids)
        clear_starts = world(clear_origins, clear_scan_ids)
        extent += [clear_ends, clear_starts]
    extent = np.vstack(extent)
    origin = np.floor((extent.min(axis=0)-margin)/resolution)*resolution
    shape_xy = np.ceil((extent.max(axis=0)+margin-origin)/resolution).astype(np.int64)+1
    width, height = map(int, shape_xy)
    if width <= 0 or height <= 0 or width * height > max_cells:
        raise ValueError(f'Grid {width}x{height} exceeds {max_cells} cells; increase resolution or inspect pose outliers')
    endpoint_cells = np.floor((endpoints-origin)/resolution).astype(np.int64)
    start_cells = np.floor((starts-origin)/resolution).astype(np.int64)
    hit_groups = {}
    for index, sid in enumerate(scan_ids.tolist()):
        hit_groups.setdefault(sid, []).append(index)
    clear_groups = {}
    if use_clear:
        clear_end_cells = np.floor((clear_ends-origin)/resolution).astype(np.int64)
        clear_start_cells = np.floor((clear_starts-origin)/resolution).astype(np.int64)
        for index, sid in enumerate(clear_scan_ids.tolist()):
            clear_groups.setdefault(sid, []).append(index)

    def accumulate(blocked):
        log_odds = np.zeros((height, width), dtype=np.float32)
        observed = np.zeros((height, width), dtype=bool)
        # Apply at most one hit/miss per cell per scan; correlated zones/frames do
        # not multiply confidence, and hits take priority over free rays in a scan.
        for sid in sorted(set(hit_groups) | set(clear_groups)):
            free, occupied = set(), set()
            for index in hit_groups.get(sid, ()):
                cells = list(line_cells(start_cells[index], endpoint_cells[index]))
                free.update(cells[:-1]); occupied.add(cells[-1])
            if blocked is not None:
                for index in clear_groups.get(sid, ()):
                    cells = list(line_cells(clear_start_cells[index], clear_end_cells[index]))
                    # Stop one cell before the first (dilated) wall cell
                    stop = next((k for k, (x, y) in enumerate(cells) if blocked[y, x]), None)
                    free.update(cells if stop is None else cells[:max(stop - 1, 0)])
            free.difference_update(occupied)
            for cells, update in ((free, miss), (occupied, hit)):
                if not cells:
                    continue
                xy = np.asarray(list(cells), dtype=int)
                x, y = xy[:, 0], xy[:, 1]
                log_odds[y, x] = np.clip(log_odds[y, x] + update, -5., 5.)
                observed[y, x] = True
        return log_odds, observed

    log_odds, observed = accumulate(None)          # rule A: hit beams only
    if use_clear:                                  # rule D: walls from pass 1 block clear beams
        log_odds, observed = accumulate(_dilate(log_odds > 0))
    return log_odds, observed, origin


def export_map(output, log_odds, observed, origin, resolution, *, metadata=None,
               occupied_threshold=.65, free_threshold=.196, preview=False):
    probability = 1 / (1 + np.exp(-log_odds))
    image = np.full(log_odds.shape, 205, dtype=np.uint8)
    image[observed & (probability < free_threshold)] = 254
    image[observed & (probability > occupied_threshold)] = 0
    # PGM top row is north; grid row zero is south (Nav2/ROS map convention).
    image = np.flipud(image)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    height, width = image.shape
    with (output / 'map.pgm').open('xb') as handle:
        handle.write(f'P5\n# local ENU; grey=unknown\n{width} {height}\n255\n'.encode('ascii'))
        handle.write(image.tobytes())
    (output / 'map.yaml').write_text(
        'image: map.pgm\nmode: trinary\n'
        f'resolution: {resolution:.12g}\n'
        f'origin: [{origin[0]:.12g}, {origin[1]:.12g}, 0.0]\n'
        # YAML thresholds decode the fixed PGM colours, independently of the
        # probability thresholds used above to classify the evidence grid.
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    np.savez_compressed(output / 'occupancy.npz', probability=probability,
                        observed=observed, log_odds=log_odds, origin=origin, resolution=resolution)
    summary = dict(frame='local ENU; x East,y North', width=width, height=height,
                   resolution_m=resolution, origin_m=origin.tolist(),
                   occupied_cells=int(np.count_nonzero(image == 0)),
                   free_cells=int(np.count_nonzero(image == 254)),
                   unknown_cells=int(np.count_nonzero(image == 205)),
                   probability_thresholds=dict(occupied=occupied_threshold, free=free_threshold),
                   source=metadata or {})
    (output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    if preview:
        from matplotlib import image as mpl_image
        mpl_image.imsave(output / 'map.png', image, cmap='gray', vmin=0, vmax=255)
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('result', type=Path, help='slam_result.npz from offline_slam.py')
    ap.add_argument('--output', type=Path, required=True, help='New directory for map.pgm/map.yaml')
    ap.add_argument('--raw', action='store_true', help='Replay uncorrected poses for comparison')
    ap.add_argument('--resolution', type=float, default=.05, help='Metres/cell (default 0.05)')
    ap.add_argument('--margin', type=float, default=.5)
    ap.add_argument('--hit-log-odds', type=float, default=.85)
    ap.add_argument('--miss-log-odds', type=float, default=-.4)
    ap.add_argument('--occupied-threshold', type=float, default=.65)
    ap.add_argument('--free-threshold', type=float, default=.196)
    ap.add_argument('--max-cells', type=int, default=4_000_000)
    ap.add_argument('--preview', action='store_true', help='Also write a PNG of the PGM raster')
    ap.add_argument('--hits-only', action='store_true',
                    help='Free space from hit beams only (rule A); default also clears along '
                         'no-target beams, stopping at walls (rule D)')
    args = ap.parse_args(argv)
    values = (args.resolution, args.margin, args.hit_log_odds, args.miss_log_odds,
              args.occupied_threshold, args.free_threshold)
    if not all(math.isfinite(v) for v in values):
        ap.error('Numeric options must be finite')
    if args.resolution <= 0 or args.margin < 0 or args.max_cells <= 0:
        ap.error('Resolution/max-cells must be positive; margin must be nonnegative')
    if args.hit_log_odds <= 0 or args.miss_log_odds >= 0:
        ap.error('Hit log odds must be positive; miss log odds must be negative')
    if not 0 < args.free_threshold < args.occupied_threshold < 1:
        ap.error('Require 0 < free-threshold < occupied-threshold < 1')
    try:
        with np.load(args.result, allow_pickle=False) as result:
            if result['schema_version'].item() != 1:
                raise ValueError('Unsupported SLAM result version')
            metadata = json.loads(str(result['metadata_json'].item()))
            poses = result['raw_poses' if args.raw else 'optimized_poses']
            has_clear = 'clear_points' in result.files
            clear = dict(clear_points=result['clear_points'], clear_origins=result['clear_origins'],
                         clear_scan_ids=result['clear_scan_ids']) if has_clear else {}
            log_odds, observed, origin = rasterize(result['points'], result['origins'],
                result['scan_ids'], poses, args.resolution, args.margin,
                args.hit_log_odds, args.miss_log_odds, args.max_cells,
                hits_only=args.hits_only, **clear)
        rule = ('rule A: hit beams only' if args.hits_only or not has_clear else
                'rule D: hit beams + no-target beams cleared to the range limit, stopping at walls')
        metadata = dict(result=str(args.result), trajectory='raw' if args.raw else 'optimized',
                        free_space=rule,
                        geometry=metadata['geometry'], slam_status=metadata['status'],
                        graph_edges=metadata['edges'], hit_log_odds=args.hit_log_odds,
                        miss_log_odds=args.miss_log_odds)
        summary = export_map(args.output, log_odds, observed, origin, args.resolution,
            metadata=metadata, occupied_threshold=args.occupied_threshold,
            free_threshold=args.free_threshold, preview=args.preview)
        print(f'Wrote {args.output / "map.pgm"} and map.yaml: '
              f'{summary["width"]}x{summary["height"]} at {args.resolution} m/cell')
        return 0
    except (OSError, ValueError, KeyError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
