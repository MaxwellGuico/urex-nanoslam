#!/usr/bin/env python3
"""Offline planar ICP + SE(2) pose-graph optimization of experiment CSVs.

No sockets or aircraft commands. Geometry must be supplied explicitly. Output
coordinates are local ENU: x East, y North, yaw CCW from East, body FLU.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree

from align_experiment import align


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def transform(points, pose):
    return np.asarray(points) @ rotation(pose[2]).T + pose[:2]


def relative(a, b):
    return np.r_[rotation(a[2]).T @ (b[:2] - a[:2]), wrap(b[2] - a[2])]


def compose(a, b):
    return np.r_[a[:2] + rotation(a[2]) @ b[:2], wrap(a[2] + b[2])]


def load_geometry(path):
    geometry = json.loads(Path(path).read_text())
    yaw = np.asarray(geometry['sensor_yaw_deg_cw'], dtype=float)
    origins = np.asarray(geometry['sensor_xy_body_frd_m'], dtype=float)
    if yaw.shape != (8,) or origins.shape != (8, 2):
        raise ValueError('Geometry requires eight sensor yaws and eight [forward,right] offsets')
    if not np.isfinite(yaw).all() or not np.isfinite(origins).all():
        raise ValueError('Non-finite sensor geometry')
    fov = float(geometry['horizontal_fov_deg'])
    if not math.isfinite(fov) or not 0 < fov < 170:
        raise ValueError('horizontal_fov_deg must be between 0 and 170')
    if type(geometry.get('column_zero_clockwise')) is not bool:
        raise ValueError('column_zero_clockwise must be true or false')
    if geometry.get('distance_model') not in ('radial', 'axial'):
        raise ValueError('distance_model must be radial or axial')
    if type(geometry.get('calibrated')) is not bool:
        raise ValueError('Geometry must explicitly declare calibrated true or false')
    return geometry


def planar_pose(row, max_tilt_rad):
    q = np.array([float(row['aligned_' + k]) for k in ('q_w', 'q_x', 'q_y', 'q_z')])
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid quaternion in aligned frame')
    w, x, y, z = q / np.linalg.norm(q)
    # Body-down vs world-down tilt; reject substantial roll/pitch for a 2D slice.
    tilt = math.acos(np.clip(1 - 2 * (x*x + y*y), -1, 1))
    if tilt > max_tilt_rad:
        return None
    yaw_ned = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    pose = np.array([float(row['aligned_ned_y']), float(row['aligned_ned_x']),
                     wrap(np.pi / 2 - yaw_ned)])
    if not np.isfinite(pose).all():
        raise ValueError('Non-finite aligned pose')
    return pose


def frame_rays(row, geometry, min_range, max_range):
    """One nearest valid central-strip return per column, in planar body FLU.

    Deliberately excludes upper/lower rows. Invalid/no-target/out-of-range zones
    produce no ray, including no free-space clearing. This is a 2D approximation.
    """
    side, sensor = int(row['side']), int(row['sensor'])
    if side not in (4, 8) or not 0 <= sensor < 8:
        raise ValueError('Unsupported sensor/grid in ToF CSV')
    yaw_cw = math.radians(geometry['sensor_yaw_deg_cw'][sensor])
    origin = np.array(geometry['sensor_xy_body_frd_m'][sensor], dtype=float) * [1, -1]
    points = []
    for col in range(side):
        values = []
        for r in (side // 2 - 1, side // 2):
            zone = r * side + col
            status = int(row[f'status_{zone:02}'])
            distance = float(row[f'distance_mm_{zone:02}']) / 1000
            if status in (5, 9) and math.isfinite(distance) and min_range <= distance <= max_range:
                values.append(distance)
        if not values:
            continue
        offset = (side / 2 - .5 - col) * math.radians(geometry['horizontal_fov_deg']) / side
        if not geometry['column_zero_clockwise']:
            offset = -offset
        distance = min(values)
        if geometry['distance_model'] == 'axial':
            distance /= math.cos(offset)
        bearing = -(yaw_cw + offset)  # FRD clockwise -> FLU counterclockwise
        points.append(origin + distance * np.array([math.cos(bearing), math.sin(bearing)]))
    return np.asarray(points, dtype=float).reshape(-1, 2), origin


def check_recording(run):
    """Never merge drones or clock-reset epochs into one graph."""
    with (run / 'telemetry.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    ids = {r.get('drone_id') for r in rows}
    boots = {r['boot_id'] for r in rows if r.get('boot_id')}
    if len(ids) != 1 or len(boots) != 1:
        raise ValueError('Use a recording with one drone and one ESP boot ID; split reset epochs first')
    previous = None
    for row in sorted(rows, key=lambda r: int(r.get('esp_send_us') or 0)):
        if not row.get('px4_pos_boot_ms'):
            continue
        stamp = int(row['px4_pos_boot_ms'])
        if previous is not None and stamp + 100 < previous:
            raise ValueError('PX4 clock reset in recording; split into separate runs first')
        previous = stamp
    return next(iter(boots)), next(iter(ids))


def flight_window(run, touchdown_height_m=0.08):
    """PX4-time window [arm, touchdown] from telemetry, in microseconds.

    Start: first sample in TAKEOFF (phase 4). End: first sample in LAND (phase 7)
    at or below touchdown_height_m above the armed height, or the last LAND sample.
    Frames on the ground after landing can carry PX4 heading resets."""
    start = end = None; ground = None; last_land = None
    with (Path(run) / 'telemetry.csv').open() as handle:
        for row in csv.DictReader(handle):
            if not row.get('px4_pos_boot_ms') or not row.get('experiment_phase'): continue
            phase = int(row['experiment_phase']); stamp = int(row['px4_pos_boot_ms']) * 1000
            if phase == 4 and start is None: start, ground = stamp, float(row['ned_z'])
            if phase == 7 and start is not None:
                last_land = stamp
                if end is None and ground - float(row['ned_z']) <= touchdown_height_m: end = stamp
    end = end or last_land
    if start is None or end is None or end <= start:
        raise ValueError('Could not find takeoff and landing phases in telemetry for --flight-only')
    return start, end


def build_scans(aligned_path, geometry, args, boot, drone, window=None):
    with Path(aligned_path).open() as handle:
        rows = list(csv.DictReader(handle))
    counts = dict(input_frames=len(rows), invalid_alignment=0, tilted_frames=0,
                  duplicate_frames=0, discarded_windows=0, aligned_frames=0, outside_flight=0)
    usable, seen, profiles = [], set(), set()
    for row in rows:
        if row.get('alignment_valid') != '1':
            counts['invalid_alignment'] += 1
            continue
        if row['boot_id'] != boot or row['drone_id'] != drone:
            raise ValueError('ToF/telemetry drone or boot mismatch')
        key = (row['boot_id'], row['sensor'], row['sequence'])
        if key in seen:
            counts['duplicate_frames'] += 1
            continue
        seen.add(key)
        if window and not window[0] <= int(row['frame_reference_px4_us']) <= window[1]:
            counts['outside_flight'] += 1
            continue
        pose = planar_pose(row, math.radians(args.max_tilt_deg))
        if pose is None:
            counts['tilted_frames'] += 1
            continue
        points, origin = frame_rays(row, geometry, args.min_range, args.max_range)
        usable.append((int(row['frame_reference_px4_us']), int(row['sensor']), pose, points, origin))
        profiles.add((int(row['side']), int(row['hz'])))
    if len(profiles) > 1:
        raise ValueError('Mixed ToF profiles; process each experiment separately')
    usable.sort(key=lambda item: item[0])
    counts['aligned_frames'] = len(usable)
    if not usable:
        raise ValueError('No frames passed alignment/tilt checks; inspect aligned.csv and timing thresholds')
    windows = {}
    first = usable[0][0]
    for item in usable:
        windows.setdefault(int((item[0] - first) // (args.scan_seconds * 1e6)), []).append(item)
    scans, origins, poses, stamps, coverage = [], [], [], [], []
    for group in windows.values():
        sensor_mask = sum(1 << sensor for sensor in {item[1] for item in group})
        reference = group[0][2]
        local_points, local_origins = [], []
        for _, _, pose, points, origin in group:
            world = transform(points, pose)
            local_points.extend((world - reference[:2]) @ rotation(reference[2]))
            start = (transform(origin[None, :], pose)[0] - reference[:2]) @ rotation(reference[2])
            local_origins.extend([start] * len(points))
        if sensor_mask.bit_count() < args.min_sensors or len(local_points) < args.min_points:
            counts['discarded_windows'] += 1
            continue
        scans.append(np.asarray(local_points)); origins.append(np.asarray(local_origins))
        poses.append(reference); stamps.append(group[0][0]); coverage.append(sensor_mask)
    if len(scans) < 2:
        raise ValueError('Fewer than two usable scan windows; inspect timing, coverage and valid ranges')
    counts['scan_count'] = len(scans)
    return scans, origins, np.asarray(poses), np.asarray(stamps), np.asarray(coverage), counts


def voxel_points(points, size=.03):
    _, indices = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(indices)]


def surface_normals(points, tree):
    """Local planar normals; isolated returns/corners are not line constraints."""
    distances, neighbors = tree.query(points, k=min(5, len(points)), distance_upper_bound=.45)
    normals = np.zeros_like(points)
    valid = np.zeros(len(points), dtype=bool)
    for index, (distance, neighbor) in enumerate(zip(distances, neighbors)):
        nearby = points[neighbor[np.isfinite(distance)]]
        if len(nearby) < 3:
            continue
        values, vectors = np.linalg.eigh(np.cov(nearby.T))
        if values[-1] > 1e-8 and values[0] / values[-1] < .1:
            normals[index] = vectors[:, 0]
            valid[index] = True
    return normals, valid


def icp(source, target, initial, *, max_distance=.25, min_points=12,
        min_overlap=.5, max_rmse=.08, max_translation=.3, max_yaw=math.radians(15), iterations=40):
    """Point-to-line ICP mapping source into target, initialized by odometry.

    Reject single-wall geometry, non-convergence, weak overlap and large
    innovations. Thresholds/edge weights are heuristics, not sensor covariances.
    """
    source, target = voxel_points(source), voxel_points(target)
    info = dict(accepted=False, reason='', metric='point_to_line', rmse=None, overlap=0., iterations=0)
    estimate = np.array(initial, dtype=float)
    def reject(reason):
        info['reason'] = reason
        return estimate, info
    if min(len(source), len(target)) < min_points:
        return reject('too_few_points')
    for points in (source, target):
        eigen = np.linalg.eigvalsh(np.cov(points.T))
        if eigen[-1] <= 1e-10 or eigen[0] / eigen[-1] < .01:
            return reject('degenerate_geometry')
    tree = cKDTree(target)
    normals, valid_normals = surface_normals(target, tree)
    converged = False
    for iteration in range(iterations):
        moved = transform(source, estimate)
        distances, indices = tree.query(moved)
        keep = (distances <= max_distance) & valid_normals[indices]
        if np.count_nonzero(keep) < min_points:
            return reject('too_few_correspondences')
        # Match line surfaces instead of requiring sparse zones to hit exactly
        # the same wall points in successive views. Trim outlying normal errors.
        normal_errors = np.abs(np.einsum('ij,ij->i', normals[indices], moved-target[indices]))
        selected = np.flatnonzero(keep & (normal_errors <= np.quantile(normal_errors[keep], .9)))
        selected = selected[np.argsort(distances[selected])]
        _, unique = np.unique(indices[selected], return_index=True)
        selected = selected[unique]
        if len(selected) < min_points:
            return reject('too_few_unique_correspondences')
        a, b, n = moved[selected], target[indices[selected]], normals[indices[selected]]
        jacobian = np.column_stack([n[:, 0], n[:, 1], -n[:, 0]*a[:, 1]+n[:, 1]*a[:, 0]])
        singular = np.linalg.svd(jacobian, compute_uv=False)
        if singular[-1] / singular[0] < 1e-3:
            return reject('unobservable_geometry')
        delta = np.linalg.lstsq(jacobian, -np.einsum('ij,ij->i', n, a-b), rcond=None)[0]
        estimate = compose(delta, estimate)
        info['iterations'] = iteration + 1
        if np.linalg.norm(delta[:2]) < 1e-4 and abs(delta[2]) < 1e-4:
            converged = True
            break
    moved = transform(source, estimate)
    distances, indices = tree.query(moved)
    keep = (distances <= max_distance) & valid_normals[indices]
    reverse_distances, _ = cKDTree(moved).query(target)
    info['overlap'] = float(min(keep.mean(), (reverse_distances <= max_distance).mean()))
    errors = np.einsum('ij,ij->i', normals[indices], moved-target[indices])
    info['rmse'] = float(np.sqrt(np.mean(errors[keep] ** 2))) if keep.any() else None
    innovation = relative(np.asarray(initial), estimate)
    info['translation_innovation_m'] = float(np.linalg.norm(innovation[:2]))
    info['yaw_innovation_deg'] = float(math.degrees(abs(innovation[2])))
    if not converged:
        return reject('not_converged')
    if info['overlap'] < min_overlap:
        return reject('low_overlap')
    if info['rmse'] is None or info['rmse'] > max_rmse:
        return reject('high_rmse')
    if np.linalg.norm(innovation[:2]) > max_translation or abs(innovation[2]) > max_yaw:
        return reject('large_innovation')
    info.update(accepted=True, reason='accepted')
    return estimate, info


def optimize_graph(raw, edges, max_evaluations=200):
    """Fix first pose (gauge); robust, sparse SE(2) least squares."""
    if len(raw) < 2:
        raise ValueError('At least two graph nodes required')
    sparsity = lil_matrix((3 * len(edges), 3 * (len(raw) - 1)), dtype=int)
    for index, edge in enumerate(edges):
        for node in (edge['i'], edge['j']):
            if node:
                sparsity[3*index:3*index+3, 3*(node-1):3*node] = 1
    def residual(values):
        poses = np.vstack([raw[0], values.reshape(-1, 3)])
        errors = []
        for edge in edges:
            predicted = relative(poses[edge['i']], poses[edge['j']])
            error = predicted - edge['measurement']
            error[2] = wrap(error[2])
            errors.extend(error / edge['sigma'])
        return np.asarray(errors)
    initial = raw[1:].ravel()
    solution = least_squares(residual, initial, jac_sparsity=sparsity.tocsr(),
                             loss='soft_l1', f_scale=1., max_nfev=max_evaluations)
    info = dict(success=bool(solution.success), message=solution.message,
                initial_squared_residual=float(np.sum(residual(initial)**2)),
                final_squared_residual=float(np.sum(residual(solution.x)**2)),
                robust_cost=float(solution.cost), evaluations=int(solution.nfev))
    optimized = np.vstack([raw[0], solution.x.reshape(-1, 3)])
    optimized[:, 2] = wrap(optimized[:, 2])
    return optimized, info


def make_graph(scans, raw, args):
    edges, attempts = [], []
    def edge(i, j, kind, measurement, translation_sigma, yaw_sigma):
        edges.append(dict(i=i, j=j, kind=kind, measurement=np.asarray(measurement),
                          sigma=np.array([translation_sigma, translation_sigma, math.radians(yaw_sigma)])))
    def match(i, j, kind):
        measured, info = icp(scans[j], scans[i], relative(raw[i], raw[j]),
                            max_distance=args.icp_distance, min_points=args.min_points,
                            min_overlap=args.icp_overlap, max_rmse=args.icp_rmse,
                            max_translation=args.max_correction,
                            max_yaw=math.radians(args.max_yaw_correction_deg))
        info.update(i=i, j=j, kind=kind)
        if info['accepted'] and kind == 'loop':
            backward, reverse_info = icp(scans[i], scans[j], relative(raw[j], raw[i]),
                max_distance=args.icp_distance, min_points=args.min_points,
                min_overlap=args.icp_overlap, max_rmse=args.icp_rmse,
                max_translation=args.max_correction, max_yaw=math.radians(args.max_yaw_correction_deg))
            consistency = compose(measured, backward)
            info['reverse_accepted'] = reverse_info['accepted']
            if not reverse_info['accepted'] or np.linalg.norm(consistency[:2]) > .05 or abs(consistency[2]) > math.radians(5):
                info.update(accepted=False, reason='reverse_consistency_failed')
        attempts.append(info)
        if info['accepted']:
            edge(i, j, kind, measured, args.icp_sigma_m, args.icp_sigma_deg)
        return info['accepted']
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1))]
    for j in range(1, len(raw)):
        edge(j-1, j, 'odometry', relative(raw[j-1], raw[j]), args.odom_sigma_m, args.odom_sigma_deg)
        match(j-1, j, 'sequential_icp')
        candidates = [i for i in range(max(0, j - args.loop_separation + 1))
                      if distance[j] - distance[i] >= args.loop_travel
                      and np.linalg.norm(raw[j, :2] - raw[i, :2]) <= args.loop_radius
                      and abs(wrap(raw[j, 2] - raw[i, 2])) <= math.radians(45)]
        candidates.sort(key=lambda i: np.linalg.norm(raw[j, :2] - raw[i, :2]))
        for i in candidates[:args.loop_candidates]:
            if match(i, j, 'loop'):
                break  # At most one loop per new node, never force start=end.
    return edges, attempts


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run', type=Path, help='Directory containing telemetry.csv and tof.csv')
    ap.add_argument('--geometry', type=Path, required=True)
    ap.add_argument('--output', type=Path, help='New output directory; default RUN/slam')
    defaults = dict(scan_seconds=.5, min_range=.05, max_range=3., max_tilt_deg=10.,
        max_pose_gap_ms=100., max_readiness_halfwidth_ms=10., icp_distance=.25,
        icp_overlap=.5, icp_rmse=.08, max_correction=.3, max_yaw_correction_deg=15.,
        loop_radius=.75, loop_travel=2., odom_sigma_m=.08, odom_sigma_deg=5.,
        icp_sigma_m=.03, icp_sigma_deg=3.)
    for name, value in defaults.items():
        ap.add_argument('--' + name.replace('_', '-'), type=float, default=value)
    ap.add_argument('--flight-only', action='store_true',
                    help='Use only frames from takeoff to touchdown (from telemetry phases)')
    ap.add_argument('--assume-unbounded-period-ms', type=float,
                    help='Opt-in: accept frames without a readiness bracket, bounded by one frame period '
                         '(66.7 at 15 Hz). Also raise --max-readiness-halfwidth-ms above half of it.')
    for name, value in dict(min_points=12, min_sensors=8, loop_separation=10,
                            loop_candidates=5, max_nodes=2000, max_evaluations=200).items():
        ap.add_argument('--' + name.replace('_', '-'), type=int, default=value)
    args = ap.parse_args(argv)
    for name in defaults:
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            ap.error(f'{name} must be finite and positive')
    if not .05 <= args.scan_seconds <= 5 or not 0 < args.icp_overlap <= 1:
        ap.error('scan-seconds must be 0.05–5 and icp-overlap must be (0,1]')
    if not 1 <= args.min_sensors <= 8 or args.min_points < 3 or args.loop_separation < 2:
        ap.error('Invalid min-sensors, min-points or loop-separation')
    if min(args.loop_candidates, args.max_nodes, args.max_evaluations) < 1 or args.min_range >= args.max_range:
        ap.error('Invalid resource limits or range interval')
    try:
        geometry = load_geometry(args.geometry)
        boot, drone = check_recording(args.run)
        output = args.output or args.run / 'slam'
        output.mkdir(parents=True, exist_ok=False)
        (output / 'geometry.json').write_text(json.dumps(geometry, indent=2) + '\n')
        if not geometry['calibrated']:
            print('Geometry is marked UNCALIBRATED; maps and ICP are preliminary.')
        period = int(args.assume_unbounded_period_ms*1000) if args.assume_unbounded_period_ms else None
        count, valid = align(args.run, int(args.max_pose_gap_ms*1000),
                             int(args.max_readiness_halfwidth_ms*1000), output / 'aligned.csv',
                             unbounded_period_us=period)
        print(f'Aligned {valid}/{count} frames; building motion-compensated scan windows.', flush=True)
        window = flight_window(args.run) if args.flight_only else None
        if window:
            print(f'Flight-only window: {(window[1]-window[0])/1e6:.1f} s (takeoff to touchdown)', flush=True)
        scans, origins, raw, stamps, coverage, counts = build_scans(
            output / 'aligned.csv', geometry, args, boot, drone, window)
        if len(raw) > args.max_nodes:
            raise ValueError('Too many nodes; increase scan-seconds or explicitly raise max-nodes')
        edges, attempts = make_graph(scans, raw, args)
        (output / 'icp_diagnostics.json').write_text(json.dumps(attempts, indent=2, allow_nan=False) + '\n')
        optimized, optimizer = optimize_graph(raw, edges, args.max_evaluations)
        edge_counts = {kind: sum(e['kind'] == kind for e in edges)
                       for kind in ('odometry', 'sequential_icp', 'loop')}
        metadata = dict(schema_version=1, frame='local ENU; x=East,y=North,yaw=CCW from East; body FLU',
            boot_id=boot, drone_id=drone, geometry=geometry, counts=counts, edges=edge_counts,
            optimizer=optimizer, status='optimized' if edge_counts['sequential_icp'] or edge_counts['loop'] else 'odometry_only',
            settings={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
            inputs_sha256={name: hashlib.sha256((args.run / name).read_bytes()).hexdigest()
                           for name in ('telemetry.csv', 'tof.csv')},
            projection='Planar central two rows, nearest valid range per column; no floor/ceiling model',
            uncertainty='Heuristic ICP gates and graph weights; not calibrated covariance or accuracy guarantees')
        (output / 'summary.json').write_text(json.dumps(metadata, indent=2, allow_nan=False) + '\n')
        if not optimizer['success']:
            raise ValueError('PGO did not converge; inspect summary.json; no corrected result was exported')
        with (output / 'trajectory.csv').open('x', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['node', 'px4_us', 'raw_east_m', 'raw_north_m', 'raw_yaw_rad',
                             'east_m', 'north_m', 'yaw_rad', 'sensor_mask'])
            writer.writerows([i, int(stamps[i]), *raw[i], *optimized[i], int(coverage[i])] for i in range(len(raw)))
        with (output / 'graph_edges.csv').open('x', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['i', 'j', 'kind', 'dx', 'dy', 'dyaw', 'sigma_x', 'sigma_y', 'sigma_yaw'])
            writer.writerows([e['i'], e['j'], e['kind'], *e['measurement'], *e['sigma']] for e in edges)
        np.savez_compressed(output / 'slam_result.npz', schema_version=np.array(1),
            points=np.vstack(scans), origins=np.vstack(origins),
            scan_ids=np.repeat(np.arange(len(scans)), [len(s) for s in scans]),
            raw_poses=raw, optimized_poses=optimized, timestamps_us=stamps,
            metadata_json=np.array(json.dumps(metadata, allow_nan=False)))
        print(f'Wrote {output / "slam_result.npz"}; {len(raw)} poses, edges={edge_counts}; {metadata["status"]}')
        if not edge_counts['loop']:
            print('No loop closure passed the gates; start/end were NOT forced together.')
        return 0
    except (OSError, ValueError, KeyError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
