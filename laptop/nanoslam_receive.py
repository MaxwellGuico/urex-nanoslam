#!/usr/bin/env python3
"""Receive the drone's post-flight NanoSLAM upload and build the map.

After landing, the drone (main/slam_bridge.c) sends its pose graph over UDP:
raw + optimised poses, the 400 ms ToF scan of every node, and the accepted ICP
constraints (wire format: main/slam_upload.h). This script

  1. collects one complete upload per run (the drone repeats it to cover
     packet loss),
  2. saves the raw packets, so a run can be rebuilt later with --replay,
  3. projects the scans exactly like the drone's scan_extract_points() and
     writes slam_result.npz in the format of offline_slam.py,
  4. runs make_occupancy_grid.py for the optimised and the raw trajectory, and
  5. draws overview.png (raw vs optimised trajectory, points, constraints).

Usage:
  python3 laptop/nanoslam_receive.py                 # listen on UDP 5008
  python3 laptop/nanoslam_receive.py --once          # stop after one run
  python3 laptop/nanoslam_receive.py --replay results/slam/<run>/packets.bin
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_occupancy_grid  # noqa: E402
from run_naming import name_for_upload, profile_label  # noqa: E402

VERSIONS = (1, 2, 3)  # 2: summary adds columns / tof_hz (4x4); 3: constraints span packets
PKT_SUMMARY, PKT_NODE, PKT_CONSTRAINTS, PKT_END = 0x20, 0x21, 0x22, 0x23

HDR = struct.Struct('<BBBBIHH')                  # 12
SUMMARY = struct.Struct('<HHBBH8ffIIHHHHHH14I')  # 120 after the header
SUMMARY_V2_EXTRA = struct.Struct('<BBBB')        # columns, tof_hz, weights, reserved
NODE_HEAD = struct.Struct('<HH3f3f')             # 28
FRAME = struct.Struct('<3f64h')                  # 140
CONS_HEAD = struct.Struct('<HH')
CONSTRAINT = struct.Struct('<hhBB3f')            # 18
END = struct.Struct('<HH')

RAM_FIELDS = ('internal_total', 'internal_free_start', 'internal_free_min', 'internal_free_min_boot',
              'internal_largest_min', 'psram_total', 'psram_free_start', 'psram_free_min',
              'slam_static_bytes', 'slam_graph_bytes', 'slam_stack_size', 'slam_stack_free_min',
              'opt_stack_size', 'opt_stack_free_min')

NUM_FRAMES, NUM_DIRS, NUM_COLS = 5, 8, 8
MIN_VALID_MM, MAX_VALID_MM = 300, 2500           # SCAN_MIN/MAX_VALID_MM

assert HDR.size + SUMMARY.size == 132
assert HDR.size + SUMMARY.size + SUMMARY_V2_EXTRA.size == 136
assert HDR.size + NODE_HEAD.size + NUM_FRAMES * FRAME.size == 740


@dataclass
class Run:
    drone_id: int
    run_id: int
    summary: dict | None = None
    nodes: dict = field(default_factory=dict)
    constraint_chunks: dict = field(default_factory=dict)  # first index -> items
    end_seen: bool = False
    packets: list = field(default_factory=list)
    last_rx: float = 0.0

    @property
    def constraints(self) -> list | None:
        """All constraints once every chunk has arrived, else None."""
        if self.summary is None or not self.constraint_chunks:
            return None
        items = [c for first in sorted(self.constraint_chunks) for c in self.constraint_chunks[first]]
        return items if len(items) >= self.summary['constraint_count'] else None

    def available_constraints(self) -> list:
        """Whatever constraint chunks arrived, in order (for partial runs)."""
        return [c for first in sorted(self.constraint_chunks) for c in self.constraint_chunks[first]]

    def complete(self) -> bool:
        return (self.summary is not None and self.constraints is not None and self.end_seen
                and all(i in self.nodes for i in range(self.summary['node_count'])))

    def missing_nodes(self) -> list[int]:
        if self.summary is None:
            return []
        return [i for i in range(self.summary['node_count']) if i not in self.nodes]


def parse_packet(data: bytes):
    """Returns (pkt_type, drone_id, run_id, payload dict) or None."""
    if len(data) < HDR.size:
        return None
    pkt_type, version, drone_id, _r, run_id, _index, _total = HDR.unpack_from(data)
    if version not in VERSIONS or pkt_type not in (PKT_SUMMARY, PKT_NODE, PKT_CONSTRAINTS, PKT_END):
        return None
    off = HDR.size
    if pkt_type == PKT_SUMMARY:
        if len(data) < off + SUMMARY.size:
            return None
        v = SUMMARY.unpack_from(data, off)
        payload = dict(node_count=v[0], constraint_count=v[1], solved=bool(v[2]),
                       sensor_count=v[3], record_interval_ms=v[4],
                       sensor_yaw_rad=list(v[5:13]), sensor_offset_m=v[13],
                       icp_ms=v[14], solve_ms=v[15], seq_accepted=v[16], seq_attempts=v[17],
                       loop_accepted=v[18], loop_attempts=v[19], pos_resets=v[20],
                       icp_method='point-to-line' if v[21] == 1 else 'point-to-point',
                       ram=dict(zip(RAM_FIELDS, v[22:36])), columns=8, tof_hz=None, weights='ETH')
        if version >= 2:
            if len(data) < off + SUMMARY.size + SUMMARY_V2_EXTRA.size:
                return None
            cols, hz, weights, _ = SUMMARY_V2_EXTRA.unpack_from(data, off + SUMMARY.size)
            payload.update(columns=cols if cols in (4, 8) else 8, tof_hz=hz,
                           weights='per-axis' if weights == 1 else 'ETH')
    elif pkt_type == PKT_NODE:
        if len(data) < off + NODE_HEAD.size + NUM_FRAMES * FRAME.size:
            return None
        v = NODE_HEAD.unpack_from(data, off)
        off += NODE_HEAD.size
        frames = []
        for _ in range(NUM_FRAMES):
            f = FRAME.unpack_from(data, off)
            off += FRAME.size
            frames.append(dict(d=f[0:3], dists=np.array(f[3:], dtype=np.int16).reshape(NUM_DIRS, NUM_COLS)))
        payload = dict(node=v[0], raw=np.array(v[2:5]), optimised=np.array(v[5:8]), frames=frames)
    elif pkt_type == PKT_CONSTRAINTS:
        count, first = CONS_HEAD.unpack_from(data, off)
        off += CONS_HEAD.size
        if len(data) < off + count * CONSTRAINT.size:
            return None
        items = []
        for _ in range(count):
            c = CONSTRAINT.unpack_from(data, off)
            off += CONSTRAINT.size
            items.append(dict(from_=c[0], to=c[1], kind='loop' if c[2] == 1 else 'seq', z=list(c[4:7])))
        payload = dict(items=items, first=first)
    else:
        node_count, _ = END.unpack_from(data, off)
        payload = dict(node_count=node_count)
    return pkt_type, drone_id, run_id, payload


def absorb(run: Run, pkt_type: int, payload: dict, data: bytes) -> None:
    run.packets.append(data)
    run.last_rx = time.monotonic()
    if pkt_type == PKT_SUMMARY:
        run.summary = payload
    elif pkt_type == PKT_NODE:
        run.nodes[payload['node']] = payload
    elif pkt_type == PKT_CONSTRAINTS:
        run.constraint_chunks[payload['first']] = payload['items']
    elif pkt_type == PKT_END:
        run.end_seen = True


# --- geometry -----------------------------------------------------------------

def ned_to_enu_pose(p):
    """(north, east, yaw cw from north) -> (east, north, yaw ccw from east)."""
    return np.array([p[1], p[0], math.atan2(math.sin(math.pi / 2 - p[2]), math.cos(math.pi / 2 - p[2]))])


def node_rays(node: dict, yaw_table, offset_m, columns=8):
    """Endpoints and sensor origins in the node's body frame, FLU (x forward,
    y left), metres. Mirrors scan_extract_points() in components/nanoslam."""
    x, y, yaw = node['optimised']
    c0, s0 = math.cos(-yaw), math.sin(-yaw)
    points, origins = [], []
    for frame in node['frames']:
        fx, fy, fyaw = x + frame['d'][0], y + frame['d'][1], yaw + frame['d'][2]
        for d in range(NUM_DIRS):
            dir_yaw = fyaw + yaw_table[d]
            cd, sd = math.cos(dir_yaw), math.sin(dir_yaw)
            ox, oy = fx + offset_m * cd, fy + offset_m * sd
            step = -(math.pi / 4) / columns  # column 0 is the most clockwise
            for col in range(columns):
                mm = int(frame['dists'][d, col])
                if mm < MIN_VALID_MM or mm > MAX_VALID_MM:
                    continue
                a = -(columns // 2) * step + step / 2 + col * step
                r = mm / 1000.0
                px = fx + (r * math.cos(a) + offset_m) * cd - (r * math.sin(a)) * sd
                py = fy + (r * math.cos(a) + offset_m) * sd + (r * math.sin(a)) * cd
                for world, out in (((px, py), points), ((ox, oy), origins)):
                    dn, de = world[0] - x, world[1] - y
                    fwd, right = c0 * dn - s0 * de, s0 * dn + c0 * de
                    out.append((fwd, -right))
    return np.asarray(points, dtype=float).reshape(-1, 2), np.asarray(origins, dtype=float).reshape(-1, 2)


# --- outputs ------------------------------------------------------------------

def ram_report(ram: dict) -> str:
    """Human-readable RAM use for one run (bytes in, KiB out)."""
    k = lambda b: f'{b / 1024:.1f} KiB'
    lines = [
        f"Internal RAM: {k(ram['internal_free_start'])} free at start, lowest {k(ram['internal_free_min'])} "
        f"during the run (used by the run: {k(ram['internal_free_start'] - ram['internal_free_min'])}; "
        f"total {k(ram['internal_total'])}; lowest since boot {k(ram['internal_free_min_boot'])}; "
        f"smallest largest-free-block {k(ram['internal_largest_min'])})",
    ]
    if ram['psram_total']:
        lines.append(f"PSRAM: {k(ram['psram_free_start'])} free at start, lowest {k(ram['psram_free_min'])} "
                     f"(total {k(ram['psram_total'])})")
    else:
        lines.append('PSRAM: none detected')
    lines.append(f"NanoSLAM: {k(ram['slam_static_bytes'])} static (solver + ICP scratch), "
                 f"{k(ram['slam_graph_bytes'])} heap (PSRAM: pose/scan storage + solver matrices)")
    lines.append(f"Stacks: recorder {ram['slam_stack_size'] - ram['slam_stack_free_min']}/{ram['slam_stack_size']} B "
                 f"peak, optimiser {ram['opt_stack_size'] - ram['opt_stack_free_min']}/{ram['opt_stack_size']} B peak")
    return '\n'.join(lines)


def write_run(run: Run, out_root: Path, *, resolution: float, preview: bool,
              experiments_root: Path = Path('results/experiments')) -> Path:
    s = run.summary
    # Same name as this flight's experiment recording (<profile>_run<N>_<date>_<time>)
    name = name_for_upload(profile_label(columns=s['columns'], hz=s['tof_hz']), datetime.now(),
                           experiments_root, out_root)
    out = out_root / name
    for k in range(2, 100):
        if not out.exists():
            break
        out = out_root / f'{name}_upload{k}'
    out.mkdir(parents=True, exist_ok=False)

    with (out / 'packets.bin').open('xb') as handle:
        for data in run.packets:
            handle.write(struct.pack('<H', len(data)) + data)

    ids = sorted(run.nodes)
    raw = np.array([ned_to_enu_pose(run.nodes[i]['raw']) for i in ids])
    opt = np.array([ned_to_enu_pose(run.nodes[i]['optimised']) for i in ids])
    points, origins, scan_ids = [], [], []
    for k, i in enumerate(ids):
        p, o = node_rays(run.nodes[i], s['sensor_yaw_rad'], s['sensor_offset_m'], s['columns'])
        points.append(p); origins.append(o); scan_ids.append(np.full(len(p), k, dtype=np.int64))
    points = np.vstack(points) if points else np.zeros((0, 2))
    origins = np.vstack(origins) if origins else np.zeros((0, 2))
    scan_ids = np.concatenate(scan_ids) if scan_ids else np.zeros(0, dtype=np.int64)
    edges = dict(odometry=max(len(ids) - 1, 0),
                 sequential_icp=sum(c['kind'] == 'seq' for c in run.available_constraints()),
                 loop=sum(c['kind'] == 'loop' for c in run.available_constraints()))
    geometry = dict(source='drone NanoSLAM upload', sensor_yaw_rad_cw=s['sensor_yaw_rad'],
                    sensor_offset_m=s['sensor_offset_m'], rows='centre rows median', columns=s['columns'], tof_hz=s['tof_hz'])
    metadata = dict(schema_version=1, frame='local ENU; x=East,y=North,yaw=CCW from East; body FLU',
                    source='onboard NanoSLAM (ICP + LS-SLAM on the ESP32-S3)',
                    drone_id=run.drone_id, run_id=run.run_id, geometry=geometry, edges=edges,
                    status='optimized' if s['solved'] else 'solve_rejected',
                    summary=s, missing_nodes=run.missing_nodes())
    np.savez_compressed(out / 'slam_result.npz', schema_version=np.array(1),
                        points=points, origins=origins, scan_ids=scan_ids,
                        raw_poses=raw, optimized_poses=opt,
                        timestamps_us=np.array([i * s['record_interval_ms'] * 1000 for i in ids]),
                        metadata_json=np.array(json.dumps(metadata)))
    (out / 'summary.json').write_text(json.dumps(metadata, indent=2) + '\n')
    with (out / 'trajectory.csv').open('x', newline='') as handle:
        w = csv.writer(handle)
        w.writerow(['node', 'raw_east_m', 'raw_north_m', 'raw_yaw_rad', 'east_m', 'north_m', 'yaw_rad'])
        w.writerows([i, *raw[k], *opt[k]] for k, i in enumerate(ids))
    with (out / 'constraints.csv').open('x', newline='') as handle:
        w = csv.writer(handle)
        w.writerow(['from', 'to', 'kind', 'z_forward_m', 'z_right_m', 'z_yaw_rad'])
        w.writerows([c['from_'], c['to'], c['kind'], *c['z']] for c in run.available_constraints())

    if len(points):
        for name, extra in (('map', []), ('map-raw', ['--raw'])):
            make_occupancy_grid.main([str(out / 'slam_result.npz'), '--output', str(out / name),
                                      '--resolution', str(resolution), *(['--preview'] if preview else []),
                                      *extra])
    try:
        draw_overview(out, raw, opt, points, scan_ids, run.available_constraints(), ids)
    except ImportError:
        print('matplotlib not installed; skipped overview.png')
    return out


def draw_overview(out, raw, opt, points, scan_ids, constraints, ids):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def world(poses):
        p = poses[scan_ids]
        c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
        return np.column_stack([c * points[:, 0] - s * points[:, 1] + p[:, 0],
                                s * points[:, 0] + c * points[:, 1] + p[:, 1]])

    fig, axes = plt.subplots(1, 2, figsize=(13, 6.5), sharex=True, sharey=True)
    index = {node: k for k, node in enumerate(ids)}
    for ax, poses, title in ((axes[0], raw, 'PX4 odometry (raw)'), (axes[1], opt, 'NanoSLAM optimised')):
        if len(points):
            w = world(poses)
            ax.scatter(w[:, 0], w[:, 1], s=1.5, c='#5b6b7a', alpha=.35, linewidths=0)
        ax.plot(poses[:, 0], poses[:, 1], '-o', ms=3, lw=1.2, color='#3b7dd8', label='trajectory')
        if poses is opt:
            for c in constraints:
                if c['from_'] in index and c['to'] in index:
                    a, b = poses[index[c['from_']]], poses[index[c['to']]]
                    ax.plot([a[0], b[0]], [a[1], b[1]], lw=.8,
                            color='#e0803a' if c['kind'] == 'loop' else '#7fbf7f')
        ax.plot(poses[0, 0], poses[0, 1], 's', color='#2e9e5b', ms=7, label='start')
        ax.set_title(title)
        ax.set_aspect('equal')
        ax.set_xlabel('East (m)')
        ax.grid(alpha=.25)
    axes[0].set_ylabel('North (m)')
    axes[1].plot([], [], color='#7fbf7f', label='sequential ICP')
    axes[1].plot([], [], color='#e0803a', label='loop closure')
    axes[1].legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'overview.png', dpi=130)
    plt.close(fig)


# --- input loops --------------------------------------------------------------

class Collector:
    def __init__(self, out_root: Path, resolution: float, preview: bool, idle_s: float,
                 experiments_root: Path = Path('results/experiments')):
        self.experiments_root = experiments_root
        self.runs: dict[tuple[int, int], Run] = {}
        self.done: set[tuple[int, int]] = set()
        self.out_root, self.resolution, self.preview, self.idle_s = out_root, resolution, preview, idle_s
        self.written: list[Path] = []

    def feed(self, data: bytes) -> None:
        parsed = parse_packet(data)
        if parsed is None:
            return
        pkt_type, drone_id, run_id, payload = parsed
        key = (drone_id, run_id)
        if key in self.done:
            return
        run = self.runs.setdefault(key, Run(drone_id, run_id))
        absorb(run, pkt_type, payload, data)
        if run.complete():
            self.finish(key, partial=False)

    def poll_idle(self) -> None:
        now = time.monotonic()
        for key, run in list(self.runs.items()):
            if run.end_seen and run.summary and now - run.last_rx > self.idle_s:
                self.finish(key, partial=True)

    def finish(self, key, *, partial: bool) -> None:
        run = self.runs.pop(key)
        self.done.add(key)
        if partial:
            print(f'Run {run.run_id} from drone {run.drone_id} is incomplete: '
                  f'missing nodes {run.missing_nodes()}'
                  f'{"" if run.constraints is not None else " and constraints"}; building what arrived')
        out = write_run(run, self.out_root, resolution=self.resolution, preview=self.preview,
                        experiments_root=self.experiments_root)
        s = run.summary
        tof = f'{s["columns"]}x{s["columns"]}' + (f' @ {s["tof_hz"]} Hz' if s['tof_hz'] else '')
        report = (f'Drone {run.drone_id} run {run.run_id}: {len(run.nodes)} nodes, ToF {tof}, '
                  f'seq ICP {s["seq_accepted"]}/{s["seq_attempts"]}, loops {s["loop_accepted"]}/{s["loop_attempts"]}, '
                  f'solve {"ok" if s["solved"] else "REJECTED"} ({s["icp_method"]} ICP {s["icp_ms"]} ms, '
                  f'{s["weights"]} weights, solve {s["solve_ms"]} ms), '
                  f'PX4 position resets removed: {s["pos_resets"]}\n' + ram_report(s['ram']) + f'\n-> {out}')
        print(report)
        (out / 'receive.log').write_text(report + '\n')
        self.written.append(out)


def replay(path: Path, collector: Collector) -> None:
    data = path.read_bytes()
    off = 0
    while off + 2 <= len(data):
        (n,) = struct.unpack_from('<H', data, off)
        collector.feed(data[off + 2: off + 2 + n])
        off += 2 + n
    for key in list(collector.runs):
        if collector.runs[key].summary:
            collector.finish(key, partial=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=5008, help='CONFIG_NANOSLAM_UPLOAD_PORT (default 5008)')
    ap.add_argument('--output', type=Path, default=Path('results/slam'), help='Parent directory for runs')
    ap.add_argument('--experiments', type=Path, default=Path('results/experiments'),
                    help='Experiment recordings; an upload is named after its flight\'s recording')
    ap.add_argument('--resolution', type=float, default=.05, help='Map metres/cell')
    ap.add_argument('--no-preview', action='store_true', help='Skip map.png')
    ap.add_argument('--once', action='store_true', help='Exit after the first run')
    ap.add_argument('--idle', type=float, default=5., help='Seconds after the last packet before '
                    'building an incomplete run')
    ap.add_argument('--replay', type=Path, help='Rebuild from a saved packets.bin instead of listening')
    args = ap.parse_args(argv)
    collector = Collector(args.output, args.resolution, not args.no_preview, args.idle, args.experiments)

    if args.replay:
        replay(args.replay, collector)
        return 0 if collector.written else 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(.5)
    print(f'Listening for NanoSLAM uploads on UDP {args.port}; land and disarm the drone to send.')
    try:
        while True:
            try:
                data, _ = sock.recvfrom(2048)
                collector.feed(data)
            except socket.timeout:
                pass
            collector.poll_idle()
            if args.once and collector.written:
                return 0
    except KeyboardInterrupt:
        return 0
    finally:
        sock.close()


if __name__ == '__main__':
    raise SystemExit(main())
