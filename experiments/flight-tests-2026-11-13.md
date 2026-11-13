# Flight tests and fixes (2026-11-13)

Drone 8, onboard NanoSLAM, 8x8 ToF at 15 Hz, 10 ms integration, 2 s pose interval, takeoff from the
middle of the rectangle's bottom edge (`--start bottom-centre`).

**Result:** two complete 3 x 5 m rectangles (runs 6 and 7) after five fixes. Run 6's map now shows
the room's interior as free space (15,470 free cells, up from 5,255) with walls unchanged (801 cells).

## Flights

| Run | Size | Firmware build | Outcome | Notes |
| --- | --- | --- | --- | --- |
| 1 | 4 x 5.5 m | `...-yaw01-free` | Killed on west leg (2.2 m N) | Wall 0.3-0.5 m from path. A firmware LAND came first: likely the health-check race (fix 4) |
| 2 | 4 x 5.5 m | `...-yaw01-free` | Killed near NE corner | Rectangle rotated -7.9 deg against the room (fix 3). SLAM 0/47 matches (fix 5) |
| 3 | 4 x 5.5 m | `...-yaw01-free` | Killed on west leg (2.9 m N) | Rotated +7.2 deg; west wall ran into the path |
| 4 | 3 x 5 m | `...-nose-icplocal` | Firmware LAND mid leg 2, then ABORT | PX4 link, clock and ToF all healthy at the time: health-check race (fix 4) |
| 5 | 3 x 5 m | `...-nose-icplocal` | Ctrl+C before takeoff | False start |
| 6 | 3 x 5 m | `...-nose-icplocal` | **DONE** | SLAM 31/59 consecutive, 16/35 loops. Rectangle square to the room |
| 7 | 3 x 5 m | `...-healthfix` | **DONE** | No SLAM upload saved (receiver not running or upload lost) |

Builds are in `build-experiments/rectangle-8x8_15-slam-big-int10ms-*`. Data: `results/experiments/`
and `results/slam/8x8_15hz_run{N}_20261113_*`. PX4 logs: QGC `Logs/Nov13/log_16..18` = runs 1-3.

## Changes

| # | Change | Where | Evidence / test |
| --- | --- | --- | --- |
| 1 | SLAM odometry yaw sigma 1 deg -> 0.1 deg: heading follows PX4, ICP corrects position only | `components/nanoslam/slam.h` | Run 16 (Nov 8): SLAM rotated poses up to 3.8 deg off PX4; now 0.10 deg. `test_run16_heading_follows_px4` |
| 2 | No-target marker: columns with nothing within 2.5 m upload as `SCAN_NO_TARGET` (-2); ICP ignores them | `components/nanoslam/scan.c` | `test_reduce_frame_no_target` |
| 2b | Map rule D (now the default): no-target beams clear free space to 2.5 m, stopping before walls seen by hits. 3-panel `map_overview.png` per run. `--rebuild` regenerates runs in place | `laptop/make_occupancy_grid.py`, `laptop/nanoslam_receive.py` | `test_clear_beams_mark_free_space_and_stop_at_walls`, round-trip and rebuild tests. All runs in `results/slam` rebuilt |
| 3 | Rectangle laid out from the drone's nose at takeoff, not PX4's north | `main/experiment_plan.c`, `main/experiment_task.c` | Mission simulator measures the rectangle in the takeoff frame (heading 0.4 rad) |
| 4 | Health-check race: ages from timestamps written by other tasks are signed, so a message stamped 1 ms after the loop's clock read counts as fresh. The firmware now logs why it lands (serial only) | `main/experiment_task.c`, `main/tof_task.c` | Simulator scenario 12 (messages 1 ms ahead) aborted before, completes now |
| 5 | ICP runs in a frame centred on the reference pose, not the world origin | `components/nanoslam/slam.c` | Run 2 replay: 0/47 -> 33/47 consecutive, 0/21 -> 12/21 loops. Room-loop test at 80 m offset |

## What we learned

- **SLAM was rotating good maps.** PX4's heading drifts at most about 0.03 deg/s. A few bad early loop
  closures (4-8 deg off) could bend a chain of 1-deg odometry edges. Replays of all ten Nov 8
  flights with 0.1 deg: heading within 0.4 deg of PX4 (was 3-9 deg), wall sharpness unchanged
  (+-0.9 cm).
- **The room's interior stayed unknown because the drone threw away "nothing there".** 59% of
  centre-row beams report no target; only 3% return beyond 2.5 m. Clearing along no-target beams
  fills the interior, but only if the beams stop at known walls. Clearing straight through erased
  39% of the wall cells (1,553 -> 951); stopping at walls keeps them (1,551).
- **A lawnmower path doesn't help with a horizontal ToF ring.** The ring sees all around. What
  matters is that every point is within about 2.5 m of the path, so a perimeter loop 1-1.5 m
  inside the walls covers a 6.3 m-wide room.
- **Without a compass, PX4's north is arbitrary.** It is wherever the nose pointed at boot, plus
  gyro drift. Takeoff headings were -0.1, -7.9 and +7.2 deg. Raw-map wall angle plus takeoff
  heading stayed within -0.9 to -3.3 deg on every flight, so the drone was placed consistently and
  only PX4's frame moved. Anything laid out in PX4 coordinates must be rotated by the takeoff heading.
- **PX4 flew well; the plan was the problem.** It tracked setpoints with about 0.25 m lag, held
  heading, used IMU0 only, and optical-flow quality stayed around 105. A 4 m-wide rectangle left
  only 0.3-0.8 m to the walls, so a few degrees of rotation or a slightly off takeoff spot was
  enough to hit them. Hence 3 x 5 m.
- **The ESP's PX4-reset detector carries offsets across flights.** After each kill, PX4's estimate
  runs away on the ground (up to 19 m), and the offsets add up: by run 2 every SLAM pose sat
  77 m from the origin. Harmless for relative motion, but it broke world-frame ICP (fix 5).
- **Unsigned millisecond ages wrap.** The MAVLink task already had a comment about this exact bug;
  the mission health check and ToF freshness check didn't. The failure is random and rare, which
  is why an unchanged retry flew fine.

## Open items

- Raise the 64-pose graph limit (fills at about 128 s).
- Rule D leaks a little free space through gaps in the south wall and the NE corner.
- Reset the reset-detector offset at the start of each SLAM run, so uploads start near zero.
- The land reason is only logged over USB serial; adding it to telemetry would make untethered
  aborts diagnosable.
- Run 6's east leg wandered about +-0.3 m near 2 m north (also in PX4's estimate). Check optical
  flow there if it repeats.
- The obstacle stop (`CONFIG_EXPERIMENT_OBSTACLE_ABORT`) is still off.
- Map tilt: the remaining -1 to -3 deg (raw map vs placement) is the ToF ring mount or placement,
  not PX4 heading.
