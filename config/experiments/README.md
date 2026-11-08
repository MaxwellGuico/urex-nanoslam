# Four staged flight experiments

All commands below run from `firmware/nus_flight_app`. Nothing here flashes or
starts a flight automatically. Build preparation is separate from executing a
mission. Use the configured `CONFIG_DRONE_ID` with `--drone-id` (the examples use
0). Stop other receivers on UDP 5005/5007: this runner records and controls through
one receiver, avoiding competing listeners.

## Requested sequence — 7 November 2026

Manual flight has been achieved. Run these four experiments in order, advancing
only after reviewing the preceding flight and its logs. Keep the same dimensions,
altitude, speed and corner holds across experiments 2–4.

| Experiment | Mission | ToF acquisition | Onboard pose graph |
| --- | --- | --- | --- |
| 1 | Takeoff, settle, hold, land | Disabled | Disabled |
| 2 | Takeoff, rectangle, land | Disabled | Disabled |
| 3 | Takeoff, rectangle, land; record raw data on laptop | Eight sensors, native 8×8 at requested 15 Hz each | Disabled |
| 4 | Same mission and recording | Eight sensors, native 4×4 at requested 30 Hz each | Disabled |

Experiments 1–2 deliberately have **no ToF obstacle checks or ToF freshness gate**.
Their firmware does not start the sensor task or initialize the sensor hardware.
Software bookkeeping remains initialized for telemetry, reporting
ToF grid/rate **0/0** and no healthy sensors, rather than pretending coverage exists.
Experiments 3–4 require all eight sensors fresh and enable obstacle abort checks.
Every experiment retains fresh PX4 telemetry, clock-sync lock, pilot takeover
handling and mission timeouts. Camera and NanoSLAM tasks are disabled throughout.

## Mission definition

Default: width **1 m east**, length **1 m north**, altitude **0.5 m above the
captured takeoff position**, speed limit **0.3 m/s**. The axes are PX4 local NED;
they are not rotated into the initial body heading. Initial yaw is held.

The onboard sequence is:

1. Require fresh PX4 position/attitude/heartbeat, disarmed state, and clock-sync
   lock. With ToF enabled, also require a fresh complete eight-sensor ring.
   Pre-stream a hold for 2 seconds.
2. Request offboard mode, arm, and rise at a commanded maximum 0.2 m/s.
3. Settle, then remain at the starting corner for 3 seconds.
4. Fly `(length, 0) → (length, width) → (0, width) → (0, 0)` relative to the
   captured takeoff point. Stop for 3 seconds at every corner, including home.
5. Send PX4 LAND and wait for disarm.

Arrival/hold requires horizontal error ≤0.10 m, horizontal speed ≤0.08 m/s,
vertical error ≤0.10 m, and vertical speed ≤0.08 m/s. The 3-second hold timer
resets if the aircraft leaves those bands. This is a measured stationary band,
not a promise of exactly zero velocity. Position setpoints advance at the requested
speed, with bounded lead over the actual aircraft; actual velocity depends on PX4
controller tuning. The 0.5 m altitude is relative to takeoff, not terrain following.

The rectangle executes on the ESP32, so Wi-Fi timing does not schedule the corners.
Clock/telemetry loss or a stage timeout aborts toward landing. With ToF enabled,
ToF loss or a nearby obstacle during horizontal flight/hold also aborts toward
landing. It does not steer around an obstacle and silently change the rectangle.
A pilot mode change after arming ends experiment
ownership without commanding offboard again. The runner's Ctrl+C requests land if
it started an autonomous mission. In experiment images, legacy LAND/HOLD commands
request an aborted landing; legacy START/GOTO are ignored. An ESP reset still
requires PX4's configured offboard-loss behavior; the ESP cannot continue a
mission across its own reset. These are implementation behaviors requiring
progressive hardware validation, not validated aircraft safety guarantees.

## Build preparation

Source your installed ESP-IDF `export.sh`, then use the profile builder:

```bash
python3 scripts/prepare_experiment.py --stage manual --profile 8x8_15 --build
```

This copies the local hardware/network configuration into an isolated, ignored
`build-experiments/manual-8x8_15/sdkconfig`. It leaves the normal `sdkconfig`
configuration selected for the normal build. Local configuration copies contain
Wi-Fi credentials and are created with mode 600. Omit `--build` to prepare only.

The stage options are `manual`, `takeoff-land`, `rectangle`, and `record`. Manual
uses a passive firmware image: it sends no flight-control setpoints, arm, mode,
or land commands. All other stages select the same onboard experiment firmware;
the laptop command chooses takeoff/land versus rectangle. NanoSLAM and camera
processing are disabled in these profiles to isolate collection timing. The
builder defaults to `--profile off` for `takeoff-land` and `rectangle`, and
`8x8_15` for `manual` and `record`. An explicit active profile keeps sensors on
in any stage. `off` is accepted only for `takeoff-land` and `rectangle`. The
laptop must select the matching profile; omitting `--record-tof` alone does not
disable sensors or their raw UDP stream.

Build products stay in the selected directory. When you choose to flash a stage,
use that build directory and its SDKCONFIG explicitly, for example:

```bash
./scripts/idf_logged.sh \
  -B build-experiments/manual-8x8_15 \
  -D SDKCONFIG="$PWD/build-experiments/manual-8x8_15/sdkconfig" \
  -p /dev/ttyACM0 flash monitor
```

Replace the serial port with the actual setup. Flashing is a
separate action you perform when ready for that stage.

## Switching experiments

The current implementation needs three firmware images: one ToF-off image shared
by experiments 1–2, one 8×8/15 Hz image for experiment 3, and one 4×4/30 Hz image
for experiment 4. Build and flash once for experiments 1–2; then reflash when
moving to 3 and again to 4. Mission type and dimensions are selected in Python.
ToF enablement, resolution and rate remain build-time settings. The runner's
`--profile` validates the flashed settings; it does not reconfigure the sensors.

## Today's four runs

All mission commands below specify `--drone-id 0`; replace it with the configured
ID. The default rectangle is 1 m east × 1 m north at 0.5 m above takeoff and a
0.3 m/s setpoint speed, with 3-second stationary holds. Choose dimensions for the
actual flight area before running, then keep them fixed. Omit `--execute` from
any autonomous command to preview it without opening sockets or sending commands.

### 1. Autonomous takeoff and landing — ToF off

```bash
python3 scripts/prepare_experiment.py --stage takeoff-land --profile off --build
```

Flash the prepared baseline image using the actual serial port:

```bash
./scripts/idf_logged.sh \
  -B build-experiments/takeoff-land-off \
  -D SDKCONFIG="$PWD/build-experiments/takeoff-land-off/sdkconfig" \
  -p /dev/ttyACM0 flash monitor
```

Verify the startup message `BASELINE FLIGHT: ToF acquisition disabled; no ToF
obstacle checks`. The runner refuses an active-ToF image with `--profile off`.
Start the first run when ready:

```bash
python3 laptop/run_experiment.py --mode takeoff-land --profile off --drone-id 0 --execute
```

Review: takeoff altitude settles, the hold lasts 3 seconds, LAND completes with
PX4 disarmed, and telemetry contains terminal phase `DONE` without resets or
unexpected mode changes. Hardware flight behavior still requires validation.

### 2. Autonomous takeoff, rectangle and landing — ToF off

**Reuse the same baseline image; no reflash is needed.**

```bash
python3 laptop/run_experiment.py --mode rectangle --profile off --drone-id 0 --execute
```

Review: correct North/East direction, all four legs and corner holds, return to
the captured starting corner, then landing/disarm. Inspect recorded position and
velocity against the planned dimensions before adding sensor workload.

### 3. Same rectangle plus eight raw ToFs — 8×8 at 15 Hz

```bash
python3 scripts/prepare_experiment.py --stage record --profile 8x8_15 --build
```

Flash `build-experiments/record-8x8_15` using **both** the matching `-B` path and
`-D SDKCONFIG="$PWD/build-experiments/record-8x8_15/sdkconfig"` in the flashing
command above. Then run:

```bash
python3 laptop/run_experiment.py --mode rectangle --drone-id 0 \
  --profile 8x8_15 --record-tof --execute
```

Review flight behavior plus all eight per-sensor measured rates, sequence gaps,
queue drops, read durations and clock-sync health in `summary.json`. The requested
15 Hz is not proof of an achieved 15 Hz; the shared bus has limited headroom.

### 4. Same rectangle plus eight raw ToFs — 4×4 at 30 Hz

```bash
python3 scripts/prepare_experiment.py --stage record --profile 4x4_30 --build
```

Flash `build-experiments/record-4x4_30` with its matching SDKCONFIG, then run:

```bash
python3 laptop/run_experiment.py --mode rectangle --drone-id 0 \
  --profile 4x4_30 --record-tof --execute
```

Review the same flight and data metrics against experiment 3. Native 4×4 frames
contain 16 original zones per sensor, never 64 duplicated/resampled zones. These
profiles acquire, timestamp, queue and transmit raw sensor frames alongside PX4
telemetry; they do not run scan matching, mapping or pose-graph computation on
the ESP32-S3. Offline alignment is a separate laptop command below.

For a larger rectangle, append `--width 2 --length 3 --altitude 0.5 --speed 0.3`
to the mission command; these are East/North dimensions in metres. Use the same
values in experiments 2–4. Native 4×4 at 60 Hz remains available for separate bus
saturation testing and is **outside today's four experiments**.

## Clock synchronization and timestamp meaning

MAVLink TIMESYNC runs at 10 Hz over the PX4 UART. The ESP computes:

`PX4 boot time in microseconds = ESP monotonic microseconds + clock_offset_us`

The implementation matches the exact outstanding request timestamp and PX4
system/component. It rejects negative or >20 ms round trips and selects the
minimum-RTT sample in a rolling 16-sample window to reduce queue-delay bias. It
reacquires after a >100 ms offset jump or >2 seconds without an accepted sample.
A lock requires ≥8 accepted samples, best RTT ≤5 ms, and a fresh accepted sample
within 2 seconds. Hardware clocks are not stepped; every record preserves the
raw ESP times and the offset used to map them into PX4 boot time.

UART receive parsing runs every 1 ms rather than every 50 ms. Flight setpoints
remain at 20 Hz. The experiment profiles request PX4 position and attitude at
100 Hz and send laptop telemetry snapshots at 100 Hz. These are requests, not a
guarantee that PX4 publishes at that rate. The sensor profiles explicitly select autonomous ranging with 5 ms integration
and read back resolution, rate, mode and integration before starting.
Original PX4 sample timestamps let the
analysis detect duplicates and gaps. The snapshot stream can omit some MAVLink
samples; it is not a raw lossless MAVLink relay.

Each native ToF frame logs:

- Start/end of the ready-header check, start/end of result readout, previous
  not-ready observation if available, and ESP send time, all in microseconds.
- Sensor stream counter, monotonic per-sensor acquisition sequence, ESP boot ID,
  firmware queue-drop count, native resolution/rate, and transfer byte count.
- Clock offset, selected RTT, lock state and sync age captured at readout.
- All original signed millimetre distances and per-zone statuses.

When a previous not-ready observation bounds the same frame, the recorder uses
the midpoint of that readiness bracket as the frame reference and records its
halfwidth. If the frame counter changes during readout, or there is no lower
bound, it uses read completion and explicitly marks readiness timing unbounded.
This is a data-ready/readout reference, **not the time of the optical exposure**.
The ULD supplies no exposure timestamp. Repeated scans are not eight simultaneous
measurements: the sensors range independently and share a serial readout bus.

RTT/2 describes a network-delay contribution under the midpoint assumption; it
is not a certified total clock/exposure error. PX4 boot timestamps in the selected
messages are quantized to milliseconds. Readiness uncertainty, sensor processing,
integration time, bus transfer, scheduling and any stale/missing PX4 samples must
also be considered. Sub-millisecond optical-frame alignment would require an
additional hardware timing reference/interrupt capture and sensor timing
characterization; this software cannot prove it from polling alone.

Autonomous missions refuse to start without sync lock and abort toward landing if
it is lost. Manual recording preserves unlocked frames with blank PX4-reference
times so that synchronization startup/loss is visible instead of fabricated.

## Multiplexer occupancy and attainable rates

The TCA9548A remains at its rated 400 kHz. The existing slim ST output configuration
is enforced in the experiment profiles: one target per zone, distance, status,
number-of-targets detected, plus mandatory driver metadata. Target count stays
because the driver uses it to mark empty zones status 255. Unused ambient,
SPAD-count, signal, sigma, reflectance and motion fields are disabled.

Mux selection is cached under the shared bus mutex, avoiding a repeated mux write
for a readiness check followed by a read on the same channel. Cache state is
invalidated on I2C errors/reset. Full reads occur only when data is ready, and
the task yields once per ring rather than inserting a 1 ms delay per sensor.
Recovery firmware writes remain chunked to let healthy sensors run between them.
Native frames are queued (64 slots) and drained independently of telemetry; no
10 Hz snapshot throttle is applied to the recording stream.

For this one-target slim driver configuration, full transfers are expected to be
320 bytes at 8×8 and 128 bytes at 4×4. The actual `transfer_bytes` field is recorded
and the startup log reports it. Even ignoring addresses, mux selection, polling
and CPU overhead, each byte consumes at least 9 I2C clock bits:

| Requested eight-sensor profile | Frame payload demand | Minimum 400 kHz occupancy |
| --- | ---: | ---: |
| 8×8 at 15 Hz | 38,400 B/s | 86.4% |
| 4×4 at 30 Hz | 30,720 B/s | 69.1% |
| 4×4 at 60 Hz | 61,440 B/s | 138.2% |

**All eight sensors at native 4×4 60 Hz cannot be fully drained through this single
400 kHz mux bus with this output format.** The 60 Hz profile deliberately remains
available as a saturation experiment; expect skips/lower observed rates. Achieving
all eight at 60 Hz needs a transport/hardware change or fewer simultaneously read
sensors, rather than an unsupported mux clock. The 8×8 15 Hz profile has limited
headroom and still needs measurement; requested rate is never reported as measured
rate.

## Files and alignment

Each invocation creates a new `results/experiments/<profile>_run<N>_<date>_<time>/` directory (e.g. `4x4_30hz_run2_20261108_162517`; N counts runs of that ToF profile that day);
`--output <new-directory>` chooses another location.

| File | Contents |
| --- | --- |
| `run.json` | Requested mission/profile, clock convention and run metadata |
| `telemetry.csv` | Original PX4 snapshots and sync/mission state with host receipt time |
| `tof.csv` | Every received native frame and timing fields (`--record-tof`) |
| `packets.jsonl` | Original received telemetry/native datagrams encoded as base64 |
| `events.jsonl` | Mission requests, phase changes, resets and host landing requests |
| `summary.json` | Observed rates, sequence/stream-counter gaps, unbounded timing, queue drops, read times and sync RTT |

UDP is not lossless. Per-sensor acquisition sequence gaps reveal missing
successful acquisitions at the laptop, while stream-counter gaps can additionally
indicate sensor frames skipped before acquisition. Counter wrap, reinitialization,
reordering and multiple lost counter cycles limit exact attribution; raw counters
are retained. Queue loss is reported separately. The final tail of a run cannot
be proved complete without a reliable delivery protocol. The run records telemetry
and ToF on separate streams; host receipt time is not used as measurement time.

Create a per-frame file with PX4 position, velocity and quaternion interpolation:

```bash
python3 laptop/align_experiment.py results/experiments/<run-directory>
```

`aligned.csv` preserves all ToF columns, adds interpolated NED position/velocity
and quaternion, and reports position/attitude bracketing gaps. It uses the stored
clock mapping, removes repeated PX4 sample timestamps, separates ESP/PX4 reset
epochs, interpolates position/velocity linearly and quaternion with shortest-path
SLERP. It does not extrapolate or bridge pose gaps >100 ms by default. Missing
brackets, unbounded readiness, unlocked clocks or readiness halfwidth >10 ms mark
`alignment_valid=0` and include a reason. Configurable analysis thresholds do not
change original measurements or make an optical exposure timestamp available.

Start by inspecting lock/RTT, all eight observed rates, transfer sizes, frame gaps,
queue drops and readiness brackets during passive tests. These measurements are
needed before judging the dataset's usable timing accuracy.

## Software verification and sources

For laptop ICP/pose-graph optimization and Nav2-style `map.pgm` + `map.yaml`
generation from experiment 3/4 CSVs, see the [offline SLAM guide](../../laptop/OFFLINE_SLAM.md).
Sensor geometry must be supplied explicitly; the provided example is uncalibrated.

Host tests exercise ToF-off baseline configuration and mission execution, packet layouts, legacy compatibility, clock outlier/stale/jump
handling, interpolation without extrapolation, passive UDP recording, autonomous
sync gating/acknowledgment and a deterministic simulation of the actual onboard
mission state machine. The latter covers normal rectangle/takeoff-land, holds,
pilot takeover, clock loss, obstacle abort and offboard-mode timeout. These tests
are not flight or sensor-accuracy validation.

```bash
python3 -m pytest -q laptop/tests/test_experiment.py laptop/tests/test_onboard_mission.py \
  laptop/test_protocol.py laptop/test_tof_heatmap.py
```

- [MAVLink TIMESYNC protocol](https://mavlink.io/en/services/timesync.html) describes
  repeated timestamp exchange, round-trip measurement and offset filtering.
- [PX4 TIMESYNC implementation](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_timesync.cpp)
  responds using PX4 boot time in nanoseconds.
- [TI TCA9548A](https://www.ti.com/product/TCA9548A) specifies the 400 kHz limit.
- [ST VL53L5CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html)
  describes native 4×4/8×8 operation. Project-local ST ULD sources establish transfer
  sizes and the target-count/status dependency used above.
- Project paper-index query: *Ultra-Lightweight Collaborative SLAM for Robot
  Swarms*, PDF page 9 (`papers/Ultra-Lightweight_Collaborative_SLAM_for_Robot_Swarms.pdf`),
  describes pairing depth and estimator pose with an ID and timestamp; *NanoSLAM*,
  PDF page 14 (`papers/2309.12008.pdf`), describes sensor/pose acquisition. These are
  research references, not competition requirements or measured guarantees for
  this ESP32 bus. The index reported `missing-or-stale`; it was queried without
  rebuilding generated artifacts. No research equation or SAFMC rule sets this
  experiment's limits; those come from the user's requested mission and transport.
