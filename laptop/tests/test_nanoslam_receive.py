"""Tests for laptop/nanoslam_receive.py.

The packets are produced by a small C program compiled against the firmware's
main/slam_upload.h, so a layout change on either side fails here."""
from __future__ import annotations

import json
import math
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'laptop'))
import nanoslam_receive as rx  # noqa: E402

C_WRITER = r'''
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include "slam_upload.h"

/* Room 3 m (north) x 2.5 m (east); drone at node i sits at north 0.8 + 0.3 i,
 * east 1.0, yaw 0. Only sensor 0 (nose) sees the north wall, column 3. */
static void hdr(slam_upload_header_t *h, unsigned t, unsigned i, unsigned n) {
    h->pkt_type = t; h->version = SLAM_UPLOAD_VERSION; h->drone_id = 8; h->reserved = 0;
    h->run_id = 4242; h->index = i; h->total = n;
}
static void put(FILE *f, const void *p, unsigned short n) { fwrite(&n, 2, 1, f); fwrite(p, n, 1, f); }

int main(int argc, char **argv) {
    const int N = 4;
    const int cols = argc > 2 ? atoi(argv[2]) : 8;
    const int nc = argc > 3 ? atoi(argv[3]) : 1;
    const int chunks = (nc + SLAM_UPLOAD_MAX_CONSTRAINTS - 1) / SLAM_UPLOAD_MAX_CONSTRAINTS;
    const int total = N + 2 + chunks;
    const int col = cols == 4 ? 1 : 3;                   /* just clockwise of centre */
    const float ang = cols == 4 ? 0.0981748f : 0.0490874f; /* 5.625 / 2.8125 deg */
    FILE *f = fopen(argv[1], "wb");
    slam_upload_summary_t s; memset(&s, 0, sizeof s);
    hdr(&s.hdr, SLAM_PKT_SUMMARY, 0, total);
    s.node_count = N; s.constraint_count = nc; s.solved = 1; s.sensor_count = 8;
    s.record_interval_ms = 1500; s.sensor_offset_m = 0.0f;
    for (int k = 0; k < 8; k++) s.sensor_yaw_rad[k] = -k * 0.78539816f;
    s.seq_accepted = 3; s.seq_attempts = 3; s.icp_method = 1;
    s.columns = (unsigned char)cols; s.tof_hz = cols == 4 ? 30 : 15;
    s.internal_total = 300000; s.internal_free_start = 120000; s.internal_free_min = 101000;
    s.slam_stack_size = 8192; s.slam_stack_free_min = 3000; s.opt_stack_size = 8192; s.opt_stack_free_min = 2500;
    put(f, &s, sizeof s);
    for (int i = 0; i < N; i++) {
        slam_upload_node_t n; memset(&n, 0, sizeof n);
        hdr(&n.hdr, SLAM_PKT_NODE, i + 1, total);
        n.node = i;
        n.raw[0] = 0.8f + 0.3f * i + 0.02f * i; n.raw[1] = 1.0f; n.raw[2] = 0.0f;
        n.optimised[0] = 0.8f + 0.3f * i; n.optimised[1] = 1.0f; n.optimised[2] = 0.0f;
        for (int fr = 0; fr < 5; fr++)
            for (int d = 0; d < 8; d++)
                for (int c = 0; c < 8; c++) n.frames[fr].dists[d][c] = -1;
        float range = (3.0f - n.optimised[0]) / cosf(ang);
        for (int fr = 0; fr < 5; fr++) n.frames[fr].dists[0][col] = (short)lrintf(range * 1000.0f);
        /* Tail sensor (facing south) sees nothing within range */
        for (int fr = 0; fr < 5; fr++) n.frames[fr].dists[4][col] = SCAN_NO_TARGET;
        put(f, &n, sizeof n);
    }
    for (int k = 0; k < chunks; k++) {
        slam_upload_constraints_t c; memset(&c, 0, sizeof c);
        int first = k * SLAM_UPLOAD_MAX_CONSTRAINTS, count = nc - first;
        if (count > SLAM_UPLOAD_MAX_CONSTRAINTS) count = SLAM_UPLOAD_MAX_CONSTRAINTS;
        hdr(&c.hdr, SLAM_PKT_CONSTRAINTS, N + 1 + k, total);
        c.count = count; c.first = first;
        for (int j = 0; j < count; j++) {
            c.items[j].from = 3; c.items[j].to = 2; c.items[j].kind = (first + j) % 2;
            c.items[j].z[0] = -0.3f; c.items[j].z[1] = 0.001f * (first + j);
        }
        put(f, &c, offsetof(slam_upload_constraints_t, items) + count * sizeof(slam_upload_constraint_t));
    }
    slam_upload_end_t e; memset(&e, 0, sizeof e);
    hdr(&e.hdr, SLAM_PKT_END, total - 1, total); e.node_count = N;
    put(f, &e, sizeof e);
    fclose(f);
    printf("%u %u %u\n", (unsigned)sizeof s, (unsigned)sizeof(slam_upload_node_t), (unsigned)sizeof e);
    return 0;
}
'''


def build_packets(tmp_path, columns=8, constraints=1):
    if shutil.which('gcc') is None:
        pytest.skip('gcc not available')
    src = tmp_path / 'writer.c'
    src.write_text(C_WRITER)
    exe = tmp_path / 'writer'
    subprocess.run(['gcc', '-std=c11', '-I', str(ROOT / 'main'), '-I', str(ROOT / 'components/nanoslam'),
                    '-I', str(ROOT / 'components/nanoslam/icp'), str(src), '-lm', '-o', str(exe)],
                   check=True)
    out = tmp_path / 'packets.bin'
    sizes = subprocess.run([str(exe), str(out), str(columns), str(constraints)], check=True, capture_output=True,
                           text=True).stdout.split()
    assert [int(v) for v in sizes] == [136, 740, 16]
    return out


@pytest.fixture()
def packets(tmp_path):
    return build_packets(tmp_path)


def test_upload_round_trip_builds_map(packets, tmp_path):
    collector = rx.Collector(tmp_path / 'runs', resolution=.05, preview=False, idle_s=1., experiments_root=tmp_path / 'exp')
    rx.replay(packets, collector)
    assert len(collector.written) == 1
    run_dir = collector.written[0]
    summary = json.loads((run_dir / 'summary.json').read_text())
    assert summary['missing_nodes'] == []
    ram = summary['summary']['ram']
    assert ram['internal_free_min'] == 101000 and ram['opt_stack_free_min'] == 2500
    log = (run_dir / 'receive.log').read_text()
    assert 'point-to-line ICP' in log
    assert 'used by the run: 18.6 KiB' in log and 'optimiser 5692/8192 B peak' in log
    assert summary['edges'] == {'odometry': 3, 'sequential_icp': 1, 'loop': 0}

    with np.load(run_dir / 'slam_result.npz') as result:
        opt = result['optimized_poses']
        pts = result['points']
        ids = result['scan_ids']
    # NED (north, east, yaw 0) -> ENU (east, north, yaw 90 deg)
    assert np.allclose(opt[0], [1.0, 0.8, math.pi / 2], atol=1e-5)
    # Each node sees the north wall (north = 3.0) once per frame
    assert len(pts) == 4 * 5
    p = opt[ids]
    c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
    world_north = s * pts[:, 0] + c * pts[:, 1] + p[:, 1]
    assert np.allclose(world_north, 3.0, atol=0.002)
    assert (run_dir / 'map' / 'map.pgm').exists()
    assert (run_dir / 'map-raw' / 'map.yaml').exists()

    # Rule D: no-target beams are kept (body frame, 2.5 m) and clear free space
    with np.load(run_dir / 'slam_result.npz') as result:
        clear = result['clear_points']
        clear_ids = result['clear_scan_ids']
        assert result['clear_origins'].shape == clear.shape
    assert len(clear) == 4 * 5 and len(clear_ids) == len(clear)
    assert np.allclose(np.hypot(clear[:, 0], clear[:, 1]), 2.5, atol=1e-3)
    assert np.all(clear[:, 0] < -2.4)                          # behind the drone (FLU x < 0)
    with np.load(run_dir / 'map' / 'occupancy.npz') as occ:
        o, res = occ['origin'], float(occ['resolution'])
        # North 0.0 is 0.8-1.7 m behind every node, so all 4 scans clear it;
        # column 3 is 2.8 deg off the tail axis, so allow +-0.15 m east
        row = int(np.floor((0.0 - o[1]) / res))
        cols = slice(int(np.floor((0.85 - o[0]) / res)), int(np.floor((1.15 - o[0]) / res)) + 1)
        assert np.any(occ['observed'][row, cols] & (occ['probability'][row, cols] < .196))
    map_summary = json.loads((run_dir / 'map' / 'summary.json').read_text())
    assert map_summary['source']['free_space'].startswith('rule D')
    assert (run_dir / 'map_overview.png').exists()


def test_4x4_upload_projects_columns(tmp_path):
    out = build_packets(tmp_path, columns=4)
    collector = rx.Collector(tmp_path / 'runs', resolution=.05, preview=False, idle_s=1., experiments_root=tmp_path / 'exp')
    rx.replay(out, collector)
    run_dir = collector.written[0]
    assert 'ToF 4x4 @ 30 Hz' in (run_dir / 'receive.log').read_text()
    with np.load(run_dir / 'slam_result.npz') as result:
        opt, pts, ids = result['optimized_poses'], result['points'], result['scan_ids']
    p = opt[ids]
    c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
    world_north = s * pts[:, 0] + c * pts[:, 1] + p[:, 1]
    assert len(pts) == 4 * 5
    assert np.allclose(world_north, 3.0, atol=0.002)


def test_constraints_split_over_packets(tmp_path):
    out = build_packets(tmp_path, constraints=130)   # 64 + 64 + 2
    collector = rx.Collector(tmp_path / 'runs', resolution=.05, preview=False, idle_s=1., experiments_root=tmp_path / 'exp')
    rx.replay(out, collector)
    run_dir = collector.written[0]
    rows = (run_dir / 'constraints.csv').read_text().splitlines()[1:]
    assert len(rows) == 130
    assert [round(float(r.split(',')[4]), 3) for r in rows] == [round(.001 * i, 3) for i in range(130)]
    summary = json.loads((run_dir / 'summary.json').read_text())
    assert summary['edges'] == {'odometry': 3, 'sequential_icp': 65, 'loop': 65}


def test_missing_node_still_builds_partial_map(packets, tmp_path):
    data = packets.read_bytes()
    kept, off = [], 0
    while off < len(data):
        (n,) = struct.unpack_from('<H', data, off)
        pkt = data[off + 2: off + 2 + n]
        off += 2 + n
        if pkt[0] == rx.PKT_NODE and struct.unpack_from('<H', pkt, 12)[0] == 2:
            continue  # drop node 2
        kept.append(struct.pack('<H', len(pkt)) + pkt)
    lossy = tmp_path / 'lossy.bin'
    lossy.write_bytes(b''.join(kept))
    collector = rx.Collector(tmp_path / 'runs', resolution=.05, preview=False, idle_s=1., experiments_root=tmp_path / 'exp')
    rx.replay(lossy, collector)
    summary = json.loads((collector.written[0] / 'summary.json').read_text())
    assert summary['missing_nodes'] == [2]


def test_rejects_unknown_version():
    pkt = struct.pack('<BBBBIHH', rx.PKT_END, 99, 8, 0, 1, 0, 1) + struct.pack('<HH', 0, 0)
    assert rx.parse_packet(pkt) is None


def test_rebuild_in_place_regenerates_outputs(packets, tmp_path):
    collector = rx.Collector(tmp_path / 'runs', resolution=.05, preview=False, idle_s=1., experiments_root=tmp_path / 'exp')
    rx.replay(packets, collector)
    run_dir = collector.written[0]
    log = (run_dir / 'receive.log').read_text()
    (run_dir / 'map_overview.png').unlink()
    (run_dir / 'map' / 'map.pgm').write_bytes(b'stale')
    assert rx.main(['--rebuild', str(run_dir), '--no-preview']) == 0
    assert (run_dir / 'map_overview.png').exists()
    assert (run_dir / 'map' / 'map.pgm').read_bytes().startswith(b'P5')
    assert (run_dir / 'receive.log').read_text() == log
    assert sorted(p.name for p in (tmp_path / 'runs').iterdir()) == [run_dir.name]   # no new folder
