"""Numerical and end-to-end offline tests; synthetic data, not flight evidence."""
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'laptop'))
from offline_slam import (icp, transform, relative, optimize_graph, frame_rays,
                          planar_pose, load_geometry, check_recording, main as slam_main)
from make_occupancy_grid import rasterize, export_map, main as map_main


def test_icp_recovers_source_to_target_transform_and_rejects_wall():
    rng = np.random.default_rng(123)
    target = np.vstack([np.column_stack([np.linspace(0, 2, 100), np.zeros(100)]),
                        np.column_stack([np.zeros(80), np.linspace(0, 1.3, 80)]),
                        rng.uniform([.4, .3], [1.4, 1.1], size=(50, 2))])
    truth = np.array([.12, -.08, .07])
    source = (target - truth[:2]) @ np.array([[np.cos(truth[2]), -np.sin(truth[2])],
                                             [np.sin(truth[2]), np.cos(truth[2])]])
    estimate, info = icp(source, target, truth + [.02, -.02, .01])
    assert info['accepted'], info
    assert estimate == pytest.approx(truth, abs=.003)
    assert np.sqrt(np.mean((transform(source, estimate) - target)**2)) < .003
    wall = np.column_stack([np.linspace(0, 2, 100), np.zeros(100)])
    _, info = icp(wall, wall, np.zeros(3))
    assert not info['accepted'] and info['reason'] == 'degenerate_geometry'
    _, info = icp(source, target, [10, 10, 0])
    assert not info['accepted']


def test_pose_graph_keeps_anchor_and_reduces_loop_error():
    raw = np.array([[0., 0., 0.], [1., 0., 0.], [2.2, 0., 0.]])
    edges = [dict(i=0, j=1, measurement=relative(raw[0], raw[1]), sigma=np.array([.2, .2, .1])),
             dict(i=1, j=2, measurement=relative(raw[1], raw[2]), sigma=np.array([.2, .2, .1])),
             dict(i=0, j=2, measurement=np.array([2., 0., 0.]), sigma=np.array([.02, .02, .02]))]
    corrected, info = optimize_graph(raw, edges)
    assert info['success']
    assert corrected[0] == pytest.approx(raw[0])
    assert abs(corrected[-1, 0] - 2) < .01
    assert info['final_squared_residual'] < info['initial_squared_residual']


@pytest.mark.parametrize('side', [4, 8])
def test_ned_enu_conversion_sensor_lever_arm_and_no_return(side):
    pose = planar_pose(dict(aligned_q_w=1, aligned_q_x=0, aligned_q_y=0, aligned_q_z=0,
                            aligned_ned_x=2, aligned_ned_y=3), .2)
    assert pose == pytest.approx([3, 2, math.pi/2])  # body forward points North
    geometry = load_geometry(ROOT / 'config/experiments/tof_geometry.example.json')
    geometry['sensor_xy_body_frd_m'][1] = [.1, .2]
    row = dict(side=side, sensor=1)
    for zone in range(side*side):
        row[f'status_{zone:02}'] = 255
        row[f'distance_mm_{zone:02}'] = 1000
    points, origin = frame_rays(row, geometry, .05, 3)
    assert len(points) == 0  # no return must not fabricate free/occupied evidence
    row[f'status_{(side//2-1)*side:02}'] = 5
    points, origin = frame_rays(row, geometry, .05, 3)
    assert origin == pytest.approx([.1, -.2])
    assert points[0, 1] < origin[1]  # col 0 clockwise => body right = FLU negative y
    world = transform(points, pose)
    assert world[0, 0] > 3.2 and world[0, 1] > 2.1


def test_nav2_pgm_thresholds_origin_and_vertical_flip(tmp_path):
    log_odds = np.array([[-5, 5], [0, -5]], dtype=float)
    observed = np.array([[True, True], [False, True]])
    output = tmp_path/'map'
    export_map(output, log_odds, observed, np.array([-1., 2.]), .1)
    header, pixels = (output/'map.pgm').read_bytes().split(b'255\n', 1)
    assert header.startswith(b'P5\n')
    assert list(pixels) == [205, 254, 254, 0]  # north row first
    config = yaml.safe_load((output/'map.yaml').read_text())
    assert config['origin'] == [-1, 2, 0]
    assert config['resolution'] == .1 and config['negate'] == 0
    assert config['image'] == 'map.pgm' and config['mode'] == 'trinary'
    # Equivalent Nav2 trinary classification of the image bytes.
    occupancy = (255 - np.frombuffer(pixels, dtype=np.uint8).astype(float)) / 255
    assert occupancy[0] > config['free_thresh'] and occupancy[0] < config['occupied_thresh']
    assert occupancy[-1] > config['occupied_thresh']
    with pytest.raises(FileExistsError):
        export_map(output, log_odds, observed, np.array([-1., 2.]), .1)
    custom = tmp_path/'custom-thresholds'
    export_map(custom, log_odds, observed, np.array([-1., 2.]), .1,
               occupied_threshold=.95, free_threshold=.9)
    config = yaml.safe_load((custom/'map.yaml').read_text())
    # Probability classification settings must never make unknown PGM grey free.
    assert config['free_thresh'] == .196 and config['occupied_thresh'] == .65


def test_raycast_keeps_unseen_cells_unknown_and_endpoint_occupied():
    # One hit per scan avoids confidence multiplication from repeated zones.
    points = np.tile([[1., 0.]], (5, 1))
    odds, seen, origin = rasterize(points, np.zeros_like(points), np.arange(5),
                                  np.zeros((5, 3)), resolution=.1, margin=.2)
    start = np.floor((np.array([0., 0.])-origin)/.1).astype(int)
    end = np.floor((np.array([1., 0.])-origin)/.1).astype(int)
    assert seen[start[1], start[0]] and odds[start[1], start[0]] < -1.5
    assert odds[end[1], end[0]] > 4
    assert not seen[-1, -1]
    with pytest.raises(ValueError, match='exceeds'):
        rasterize(points, np.zeros_like(points), np.arange(5), np.zeros((5, 3)), max_cells=1)


@pytest.mark.parametrize('change', ['drone', 'esp_boot', 'px4_reset'])
def test_recording_rejects_mixed_drones_or_resets(tmp_path, change):
    rows = [dict(drone_id='0', boot_id='42', esp_send_us=1000, px4_pos_boot_ms=1000),
            dict(drone_id='0', boot_id='42', esp_send_us=2000, px4_pos_boot_ms=1010)]
    if change == 'drone': rows[1]['drone_id'] = '1'
    if change == 'esp_boot': rows[1]['boot_id'] = '43'
    if change == 'px4_reset': rows[1]['px4_pos_boot_ms'] = 10
    with (tmp_path/'telemetry.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    with pytest.raises(ValueError):
        check_recording(tmp_path)


def write_synthetic_run(run):
    """A drifting estimated trajectory viewing a fixed asymmetric box.

    The actual sensor data do not drift. Idealized synchronous 4x4 frames are
    synthetic, not a claim about hardware rates/timing or calibrated optics.
    """
    run.mkdir()
    geometry = load_geometry(ROOT / 'config/experiments/tof_geometry.example.json')
    telemetry, frames = [], []
    corners = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0.]])  # N,E
    trajectory = [corners[k]*(1-f)+corners[k+1]*f for k in range(4) for f in np.linspace(0, 1, 20, endpoint=False)]
    trajectory.append(corners[-1])
    for tick, true_ne in enumerate(trajectory):
        stamp = 1000 + 100*tick
        drift = tick/(len(trajectory)-1)
        estimated = true_ne + drift*np.array([.08, .04])
        yaw = .02*drift
        telemetry.append(dict(drone_id=0, boot_id=42, esp_send_us=stamp*1000-10000,
            px4_pos_boot_ms=stamp, px4_att_boot_ms=stamp, sync_locked=1,
            px4_pos_age_ms=1, px4_att_age_ms=1, ned_x=estimated[0], ned_y=estimated[1],
            ned_z=-.5, ned_vx=0, ned_vy=0, ned_vz=0,
            q_w=math.cos(yaw/2), q_x=0, q_y=0, q_z=math.sin(yaw/2)))
        for sensor in range(8):
            row = dict(drone_id=0, boot_id=42, side=4, hz=30, sensor=sensor,
                sequence=tick+1, frame_reference_esp_us=stamp*1000-10000,
                frame_reference_px4_us=stamp*1000, sync_locked=1,
                reference_method='ready_bracket_midpoint', readiness_halfwidth_us=1000)
            for zone in range(16):
                col, r = zone % 4, zone // 4
                bearing = math.radians(geometry['sensor_yaw_deg_cw'][sensor]+(1.5-col)*45/4)
                direction = np.array([math.cos(bearing), math.sin(bearing)])
                distances = []
                for axis, low, high in ((0, -2.4, 2.3), (1, -2., 2.5)):
                    if abs(direction[axis]) > 1e-6:
                        distance = ((high if direction[axis] > 0 else low)-true_ne[axis])/direction[axis]
                        if distance > 0:
                            distances.append(distance)
                distance = min(distances)
                row[f'distance_mm_{zone:02}'] = round(distance*1000)
                row[f'status_{zone:02}'] = 5 if r in (1, 2) else 255
            frames.append(row)
    for name, rows in (('telemetry.csv', telemetry), ('tof.csv', frames)):
        with (run/name).open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
    return geometry


def test_csv_to_icp_pgo_and_nav2_map_end_to_end(tmp_path):
    run = tmp_path/'synthetic'
    write_synthetic_run(run)
    geometry = ROOT/'config/experiments/tof_geometry.example.json'
    assert slam_main([str(run), '--geometry', str(geometry), '--max-range', '4']) == 0
    summary = json.loads((run/'slam/summary.json').read_text())
    assert summary['optimizer']['success']
    assert summary['edges']['sequential_icp'] > 0
    assert summary['edges']['loop'] > 0
    with np.load(run/'slam/slam_result.npz', allow_pickle=False) as result:
        raw, corrected = result['raw_poses'], result['optimized_poses']
        assert np.isfinite(corrected).all()
        assert corrected[0] == pytest.approx(raw[0])
        assert np.linalg.norm(corrected[-1, :2]-corrected[0, :2]) < np.linalg.norm(raw[-1, :2]-raw[0, :2])
    assert map_main([str(run/'slam/slam_result.npz'), '--output', str(run/'nav2'), '--preview']) == 0
    data = json.loads((run/'nav2/summary.json').read_text())
    assert data['occupied_cells'] > 0 and data['free_cells'] > 0 and data['unknown_cells'] > 0
    assert (run/'nav2/map.pgm').read_bytes().startswith(b'P5\n')
    assert (run/'nav2/map.png').exists()
    # Overwrite protection on both stages, without modifying input CSVs.
    assert slam_main([str(run), '--geometry', str(geometry)]) == 1
    assert map_main([str(run/'slam/slam_result.npz'), '--output', str(run/'nav2')]) == 1
