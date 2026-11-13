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
The 2026-11-13 flight tests and fixes (heading weight, no-target free space, ICP frame, mission
layout, health-check race) are in
[`experiments/flight-tests-2026-11-13.md`](../../experiments/flight-tests-2026-11-13.md).

## Differences from ETH NanoSLAM

Summary of everything that behaves differently from ETH's gap9-app, with the evidence for each.
Details follow in the sections below.

| # | ETH NanoSLAM | This implementation | Why |
| --- | --- | --- | --- |
| 1 | Point-to-point ICP (SVD) | Point-to-line ICP (`CONFIG_NANOSLAM_ICP_POINT_TO_LINE`, default on) | Point-to-point slides scans along walls (2.4 deg / 81 mm on resampled walls); maps 0.3-0.4 cm sharper on real flights |
| 2 | Odometry edges computed from the current pose estimates | Odometry edges from the raw (jump-free) PX4 poses | In flight the graph is solved, committed and solved again every 5 poses; ETH's form re-counts each earlier correction. Replay: halves the map twist on big flights |
| 3 | ICP edges 20x the information of odometry, in every axis (x, y, yaw alike) | Per-axis: odometry 8 cm / 0.1 deg, ICP 3 cm / 3 deg (`CONFIG_NANOSLAM_ETH_WEIGHTS` restores ETH) | PX4's gyro heading (<= 0.03 deg/s drift) is much better than one ICP match (+-1-2 deg, outliers to 8 deg); ETH's weights let ICP noise rotate whole maps 3-7 deg. Heading now follows PX4; ICP corrects position |
| 4 | Loop-closure constraints only; their off-diagonal Hessian entries are inserted (overwrite) | ICP constraints between consecutive scans as well as loop closures; entries accumulated | A consecutive-scan constraint shares its pose pair with an odometry edge; overwriting gave -20 instead of -21 and a wrong step |
| 5 | Sparse capacity 9 x MAX_SIZE; overflow halts the processor | Full lower triangle in PSRAM; overflow detected, solve rejected | A real 34-pose flight needed 1748 > 1728 entries |
| 6 | At most 50 constraints | 128, uploaded in chunks | A 43-pose flight produced at least 64 constraints; a lower cap drops loop closures |
| 7 | 8x8 only, 4 sensors | 8 sensors, 8x8 or 4x4; headings from the ToF angle table | Sensor ring and 4x4 profile support |
| 8 | Runs on a GAP9 co-processor; the STM32 host requests scan matches and reads poses back over SPI | Runs on the ESP32-S3 flight MCU; graph uploaded after landing, map built on the laptop | Project hardware and design |
| 9 | ICP on world-frame points (rotation about the world origin) | ICP in a frame centred on the reference pose | PX4's origin drifts with removed position resets; on 2026-11-13 run 2 every pose sat ~77 m from it and point-to-line rejected all 68 matches as unobservable. Replay of that flight: 0/47 -> 33/47 consecutive, 0/21 -> 12/21 loops. Test: `test_end_to_end_room_loop_at(..., 80.0f)` |
| 10 | Beams with no return are dropped | Columns with nothing within 2.5 m in >= 2 centre rows are uploaded as `SCAN_NO_TARGET` (-2); ICP ignores them | Lets the laptop map mark the room's interior free (free space up to 2.5 m, stopping at known walls) |

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
consistently biased (sparse scans around takeoff).

**2026-11-13: odometry yaw sigma 1 deg -> 0.1 deg.** Run 16 (first flight with the per-axis
weights) still drifted off PX4's heading: PX4 stayed within 0.6 deg over the flight, but the
onboard solve rotated poses by up to 3.8 deg. Three early loop closures (6->1, 7->2, 8->2)
disagreed with PX4 by 3.9, 4.2 and 7.7 deg, and a chain of 1-deg odometry edges let them bend
the heading. 0.1 deg per 2 s edge matches PX4's measured drift (~0.06 deg per edge). Final-graph
replays of all ten 2026-11-08 flights against the onboard results as flown (runs 1-15 with ETH
weights, run 16 with 1 deg): the heading stays within 0.4 deg of PX4 (was 3-9 deg), the map's wall
angle stays within ~1.5 deg of the raw PX4 map (was up to 6.7 deg off), and wall sharpness is
unchanged within +-0.9 cm (all-wall spread, a coarser metric than the table above; run 16
6.5 -> 5.6 cm, run 6 5.3 -> 5.9 cm, the rest within 0.5 cm). The trade-off: SLAM can no longer correct a drifting PX4 heading (as
with IMU1's 0.3-0.5 deg/s drift on 2026-11-07), so check PX4's heading first if a map is curved.
Rotation already present in the raw PX4 map (heading at arm, ToF ring mount) is unaffected.

Unit tests: `test_weights_reproduce_eth`, `test_per_axis_weights_keep_good_heading` (ETH 4.9 deg
vs per-axis 0.01 deg heading error), `test_repeated_passes_do_not_compound`,
`test_run16_heading_follows_px4` (run 16's real graph: 0.10 deg heading change, ICP still moves
poses by up to 24 cm).

## ICP method

`CONFIG_NANOSLAM_ICP_POINT_TO_LINE` (default on) uses `icp/icp_line.c`, a port of the
point-to-line ICP in `laptop/offline_slam.py`: it fits local wall lines in the reference scan and
minimises each point's distance to those lines, and rejects single-wall, low-overlap or poor-fit
matches. ETH's point-to-point ICP (`icp/icp.c`) pulls points onto individual old points;
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
