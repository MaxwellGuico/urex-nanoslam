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
| `scan.c` | Reduce each 8x8 frame to one range per column (median of rows 2-5, rows must agree within 100 mm), project 5 frames x 8 sensors x 8 columns to points |
| `slam_graph.c` | Up to 64 poses and their scans (scans in PSRAM) |
| `icp/icp_line.c` | Point-to-line ICP (default), ported from `laptop/offline_slam.py` |
| `icp/icp.c` | Point-to-point ICP with SVD (ETH/Friess), selectable |
| `slam.c` | ICP constraints between consecutive scans and to the closest earlier pose (loop closure), gated on correction size and inlier ratio, then `ls_slam` |
| `ls-slam/` | ETH Gauss-Newton solver: sparse H, reverse Cuthill-McKee ordering, sparse Cholesky, 3 iterations |

## Changes from Duncan's version

- Constraint off-diagonal Hessian entries are accumulated, not overwritten. A sequential-ICP
  constraint shares its pose pair with an odometry edge; overwriting gave `-20` instead of `-21`
  and a wrong step (ETH only had loop constraints, so it never hit this).
  `test_sequential_constraints_combine_with_odometry` covers it.
- Sparse matrix capacity overflow is detected; `ls_slam` then returns false and leaves the poses
  unchanged, instead of silently dropping entries.
- Solver matrices hold a full lower triangle (cannot overflow; ETH's 9 x MAX_SIZE overflowed on a
  real 34-pose flight) and are heap-allocated in PSRAM instead of ~30 KB of stack.
- Sensor headings come from the ToF task's angle table; acquisition moved to `main/`.
- Single drone: the swarm ID/broadcast layer and the onboard occupancy grid are not ported. The
  map is built on the laptop (`laptop/nanoslam_receive.py`).

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
