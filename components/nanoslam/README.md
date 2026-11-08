# NanoSLAM component

Onboard graph SLAM for the ESP32-S3: ICP scan matching plus sparse least-squares pose-graph
optimisation. Hardware-independent; `main/slam_bridge.c` drives it from the flight tasks.

## Origin and license

The code is adapted from [ETH-PBL/NanoSLAM](https://github.com/ETH-PBL/NanoSLAM) (gap9-app,
Copyright (C) 2023 ETH Zurich, GPL-3.0; pinned reference in `../../../../references/NanoSLAM`)
via Duncan Lau's ESP32-S3 port (`SLAM-SAFMC`, branch `trying-to-implement-slam-test-1`, folder
`combine_2`, commit `f90097b`). It is distributed under the
[GNU GPL v3](https://www.gnu.org/licenses/gpl-3.0.html). Binaries built from this firmware must
be accompanied by, or offer, the corresponding source.

## Pipeline

| File | Role |
| --- | --- |
| `scan.c` | Reduce each frame to one range per column (median of the centre rows: 2-5 of 8x8, 1-2 of 4x4; rows must agree within 100 mm), project 5 frames x 8 sensors x 8 (or 4) columns to points |
| `slam_graph.c` | Up to 64 poses, their raw odometry poses and scans (scans in PSRAM) |
| `icp/icp_line.c` | Point-to-line ICP (default), ported from `laptop/offline_slam.py` |
| `icp/icp.c` | Point-to-point ICP with SVD (ETH/Friess), selectable |
| `slam.c` | ICP constraints between consecutive scans and to the closest earlier pose (loop closure), gated on correction size and match quality, then `ls_slam_odometry` with raw-odometry edges and per-axis weights |
| `ls-slam/` | ETH Gauss-Newton solver: sparse H, reverse Cuthill-McKee ordering, sparse Cholesky, 3 iterations; per-axis edge weights added |

ToF settings that matter for SLAM are recorded in
[`experiments/tof-integration-time-2026-11-08.md`](../../experiments/tof-integration-time-2026-11-08.md).

## Differences from ETH NanoSLAM and Duncan's port

Summary of everything that behaves differently from ETH's gap9-app solver and Duncan's
`combine_2` port, with the evidence for each. Details follow in the sections below.

| # | ETH / Duncan | This implementation | Why |
| --- | --- | --- | --- |
| 1 | Point-to-point ICP (SVD) | Point-to-line ICP (`CONFIG_NANOSLAM_ICP_POINT_TO_LINE`, default on) | Point-to-point slides scans along walls (2.4 deg / 81 mm on resampled walls); maps 0.3-0.4 cm sharper on real flights |
| 2 | Odometry edges computed from the current pose estimates | Odometry edges from the raw (jump-free) PX4 poses | In flight the graph is solved, committed and solved again every 5 poses; ETH's form re-counts each earlier correction. Replay: halves the map twist on big flights |
| 3 | ICP edges 20x the information of odometry, in every axis (x, y, yaw alike) | Per-axis: odometry 8 cm / 1 deg, ICP 3 cm / 3 deg (`CONFIG_NANOSLAM_ETH_WEIGHTS` restores ETH) | PX4's gyro heading (~0.03 deg/s drift) is much better than one ICP match (+-1-2 deg); ETH's weights let ICP noise rotate whole maps 3-7 deg |
| 4 | Constraint off-diagonal Hessian entries inserted (overwrite) | Accumulated | A sequential constraint shares its pose pair with an odometry edge; overwriting gave -20 instead of -21 and a wrong step (Duncan's code only; ETH had no sequential constraints) |
| 5 | Sparse capacity 9 x MAX_SIZE, overflow silently drops entries | Full lower triangle in PSRAM; overflow detected, solve rejected | A real 34-pose flight needed 1748 > 1728 entries |
| 6 | Solver matrices on the stack (~30 KB, Duncan) | Static / PSRAM | An 8 KB optimiser stack overflowed mid-flight (2026-11-07) |
| 7 | At most 64 constraints | 128, uploaded in chunks | A 43-pose flight hit 64 exactly, dropping loop closures |
| 8 | 8x8 only, 4 sensors rotated (ETH) / fixed headings (Duncan) | 8x8 or 4x4; headings from the ToF angle table | 4x4 profile support |
| 9 | Swarm graph, onboard occupancy grid (Duncan) | Single drone; graph uploaded after landing, map built on the laptop | Project design |

### Edge weighting and odometry (items 2 and 3)

Host replays of four 2026-11-08 8x8 flights with the drone's in-flight schedule (optimise and
commit every 5 poses, final pass after landing). Map rotation = mean wall angle relative to
PX4's frame; PX4's own heading was stable within ~2 deg on every flight. The "old" replay
reproduces what the drone actually produced in flight to 0.1 deg.

| Flight | Raw PX4 rotation | Old (ETH) | Raw odometry, ETH weights | Raw odometry + per-axis | Wall spread old / new |
| --- | --- | --- | --- | --- | --- |
| Small room, run 1 | +0.7 deg | -2.9 deg | -2.9 deg | **-0.7 deg** | 6.6 / 6.6 cm |
| Small room, run 2 | -4.8 deg | -5.7 deg | -6.0 deg | **-5.2 deg** | 6.6 / 6.6 cm |
| 4 x 5.5 m, run 7 (5 ms) | -1.2 deg | -6.2 deg | -3.3 deg | **-1.8 deg** | left wall 5.9 / 6.3 cm |
| 4 x 5.5 m, run 15 (10 ms) | -1.2 deg | -7.0 deg | -4.6 deg | **-4.5 deg** | left wall 5.4 / 5.4 cm |

Run 15 keeps about 3 deg of residual rotation; the remaining ICP heading evidence there is
consistently biased (sparse scans around takeoff). Unit tests: `test_weights_reproduce_eth`,
`test_per_axis_weights_keep_good_heading` (ETH 4.9 deg vs per-axis 0.5 deg heading error),
`test_repeated_passes_do_not_compound`.

## ICP method

`CONFIG_NANOSLAM_ICP_POINT_TO_LINE` (default on) uses `icp/icp_line.c`, a port of the
point-to-line ICP in `laptop/offline_slam.py`: it fits local wall lines in the reference scan and
minimises each point's distance to those lines, and rejects single-wall, low-overlap or poor-fit
matches. ETH/Duncan's point-to-point ICP (`icp/icp.c`) pulls points onto individual old points;
because ToF zones hit different spots on a wall from each pose, it slides scans along walls (on
resampled synthetic walls: 2.4 deg / 81 mm error vs 0 for point-to-line). One deliberate
difference from the laptop version: a match whose final update is below 5 mm and 0.3 deg counts
as converged, because on real flights most strict-threshold failures were good matches
alternating at the millimetre level.

Host replay of the two 2026-11-08 rectangle flights (wall spread, lower is sharper):

| Flight | Raw PX4 | Point-to-point | Point-to-line |
| --- | --- | --- | --- |
| 1 | 8.8 cm | 6.8 cm | 6.5 cm |
| 2 | 7.9 cm | 7.0 cm | 6.6 cm |

## Tests

```bash
cmake -S components/nanoslam -B /tmp/nanoslam-build
cmake --build /tmp/nanoslam-build && ctest --test-dir /tmp/nanoslam-build --output-on-failure
```
