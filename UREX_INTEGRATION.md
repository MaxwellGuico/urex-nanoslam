# UREX NanoSLAM integration

This directory is based on the public NUS SAFMC flight application at
<https://github.com/nus-safmc/esp-everything>, pinned at commit
`99cde05b1acf568a86a642451af3e7f19a6af048` when imported.

The upstream repository did not contain a top-level licence file when it was
imported. Keep this working copy internal unless the upstream licensing terms
are clarified.

## Current NanoSLAM milestone

NanoSLAM-Lite was removed on 2026-11-07 (it remains in git history). The firmware now runs
NanoSLAM as ETH and Duncan Lau built it: `components/nanoslam` (ICP + sparse LS-SLAM, GPL-3.0,
see its README) driven by `main/slam_bridge.c`, which

- starts a fresh graph when the vehicle arms;
- every `CONFIG_NANOSLAM_RECORD_INTERVAL_MS` (1.5 s) stores the jump-free PX4 pose
  (`slam_x/slam_y`: estimator position resets removed in `mavlink_task.c`) and a 400 ms scan of
  five ring snapshots, dropping frames older than 100 ms or smeared by yaw rate;
- every `CONFIG_NANOSLAM_INFLIGHT_EVERY_N` (5) poses runs ICP between consecutive scans and to the
  closest earlier pose, solves the graph in a background task on core 0 at priority 1, and
  commits the result unless it moves the newest pose by more than 0.5 m;
- after disarming runs a final optimisation and uploads raw poses, optimised poses, scans, and
  constraints to the laptop over UDP (`main/slam_upload.h`), where
  `laptop/nanoslam_receive.py` builds the occupancy map and a Nav2 `map.pgm`.

It does **not** send MAVLink messages, alter setpoints, change the vehicle's flight estimator, or
feed the corrected pose into navigation (`slam_bridge_odom_to_map()` exists but is unused).

The ToF task keeps independent VL53L5CX driver state and work buffers for all eight mux
channels, transfers only the distance/target outputs consumed by SLAM, and
bounds I2C transactions to 100 ms. Both mux-control and downstream sensor
transactions use the TCA9548A's rated 400 kHz limit. The ring runs at 10 Hz
to leave bus-bandwidth margin. Startup configures all eight channels while
ranging is stopped, starts them in one bounded sequence, and immediately
sweeps the short readiness headers before transferring complete frames. A
channel is not published online until its first full frame arrives. I2C
failures are logged before conversion to the ST API status byte, including
sensor ID, mux channel, operation, register, full `esp_err_t`, and
`esp_err_to_name()`. A sensor is invalidated after three seconds
without a fresh frame and queued to a lower-priority recovery worker. Mux
selection and each sensor transfer are atomic, while large recovery writes
are divided into 256-byte transactions so healthy channels continue being
polled between recovery chunks. Stale mapping data is never retained.

`CONFIG_BENCH_MODE` defaults to enabled. In that configuration, the navigation and mission tasks
are not spawned and no setpoints are published. MAVLink telemetry, ToF, Wi-Fi, AprilTag/camera
detection, and NanoSLAM remain active.

Wi-Fi initialization is non-blocking. If the configured access point is not
available, its event handler continues reconnecting while MAVLink, ToF, camera,
and NanoSLAM start normally. The original blocking initialization prevented
all sensing and localization tasks from starting during the first bench test.

The connected XIAO ESP32S3 Sense reports 8 MB of physical flash. The imported
configuration declared 2 MB, so the UREX configuration now declares 8 MB and
removes the boot-time size mismatch warning.

The first complete bench boot received an initial PX4 pose but the position and
attitude streams then became stale. The MAVLink task now requests
`LOCAL_POSITION_NED` and `ATTITUDE` at 20 Hz using
`MAV_CMD_SET_MESSAGE_INTERVAL`, retries the request only while either stream is
stale, and reports five-second receive counts and message ages. This command
changes telemetry publication only; it does not arm the vehicle or enable
offboard mode.

ESP-IDF 6 splits the I2C and GPIO APIs into `esp_driver_i2c` and
`esp_driver_gpio`; the imported VL53L5CX component's CMake requirements include
those components explicitly for IDF 6 compatibility.

The main component likewise declares the split GPIO, I2C, and UART driver
components required by its public headers when building with ESP-IDF 6.

The pinned `esp32-camera` 2.1.5 component also uses `hal/clk_gate_ll.h` on
ESP32-S3 with IDF 6.0, so its private requirements include `esp_hal_clock` for
IDF 6.0 and newer. This preserves the pinned camera code rather than silently
upgrading it during the SLAM integration.

The review configuration leaves `CONFIG_APRILTAG_CAMERA_ENABLED` disabled. MAVLink, ToF, Wi-Fi,
and NanoSLAM remain pinned to Core 0; bench mode does not start navigation or mission tasks,
so Core 1 has no project task. The AprilTag task and the camera driver's internal worker are both
assigned to Core 1 when camera support is enabled later.

## Props-off acceptance test

1. Start `python3 laptop/nanoslam_receive.py` on the laptop at `CONFIG_HOST_IPV4_ADDR`.
2. Boot the ESP32-S3 with propellers removed; confirm `NanoSLAM ready` and the MAVLink `RX/5s`
   line with position and attitude ages below 250 ms.
3. For a hand-carried test without arming, build with `CONFIG_NANOSLAM_RECORD_DISARMED=y`;
   otherwise arm (props off) to start a run and confirm `New SLAM run`.
4. Confirm a `Pose n/64` line every 1.5 s and `Scan for pose` lines with `kept` near 320 and few
   `stale` columns.
5. Carry the vehicle around a measured loop. Every five poses confirm `SLAM pass ... finished`,
   and inspect accepted loop constraints against the recorded ground truth.
6. Disarm (or fill the graph in disarmed mode) and confirm `uploaded to <laptop>`; the receiver
   prints the run directory.
7. Compare `overview.png`, `map/map.png` and `map-raw/map.png` against the measured geometry.

Do not let NanoSLAM influence flight navigation yet. Execution time, false-match rate, and map
accuracy still need props-off and flight evidence.
