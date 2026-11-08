# UREX ESP32-S3 flight firmware

This repository contains the current ESP-IDF firmware for the UREX/NUS SAFMC drone. The review
snapshot integrates PX4 MAVLink telemetry, an eight-sensor VL53L5CX ToF ring, AprilTag camera
processing, Wi-Fi telemetry, and onboard NanoSLAM (ICP + pose-graph optimisation, ported from
ETH NanoSLAM via Duncan Lau's ESP32-S3 firmware).

> **Safety:** NanoSLAM is not flight-validated. It is read-only (it never sends setpoints and does
> not feed navigation). Keep `CONFIG_BENCH_MODE=y` for props-off work.

## Repository layout

| Path | Purpose |
| --- | --- |
| `main/` | ESP32-S3 application tasks and hardware integration |
| `components/nanoslam/` | NanoSLAM (GPL-3.0): ICP, sparse LS-SLAM solver, scan extraction, pose graph, host tests |
| `components/rjrp44__vl53l5cx/` | VL53L5CX driver with the project's ESP-IDF integration changes |
| `components/espressif__esp32-camera/` | Camera driver used by the AprilTag task |
| `components/esp-apriltag/` | AprilTag detector |
| `components/mavlink/` | MAVLink component and generated C headers |
| `laptop/` | Host-side mission, relay, visualization, and diagnostic tools |
| `experiments/` | Bench procedure and the latest serial logs, grouped by ToF power source |
| `UREX_INTEGRATION.md` | Integration status and known limitations |

Generated build output, editor databases, Python caches, and `sdkconfig.old` are intentionally
excluded from version control.

## Rectangle missions and timestamped recording

See [the staged experiment guide](config/experiments/README.md) for manual flight,
autonomous takeoff/landing, a rectangle with three-second corner holds and return
home/landing, and native ToF recording at 8×8 15 Hz and 4×4 30 Hz. The four-run
sequence disables ToF acquisition for the first two flights and enables all eight
sensors for the recording flights; NanoSLAM and camera tasks stay disabled.
The default mission is 1 m × 1 m, 0.5 m above takeoff, 0.3 m/s; dimensions are
command-line options. The 60 Hz eight-sensor profile exceeds the single 400 kHz
mux bus's payload capacity and is provided as a saturation experiment.

The normal firmware remains in bench mode. Isolated profiles distinguish passive
manual recording from explicitly commanded autonomous flight. No profile is
flashed automatically. The experiment firmware runs the full mission onboard;
Wi-Fi does not schedule its corners. Ports remain 5005 (configurable telemetry),
5006 (commands), and 5007 (native ToF plus optional legacy heatmap packets).

Telemetry now preserves the original 55-byte prefix, the 125-byte PX4 extension,
and an additive timing/mission extension (165 bytes total). The decoder still
accepts legacy packets. It reports position/velocity, attitude, original PX4 sample
timestamps, MAVLink TIMESYNC offset/RTT/lock, ESP microsecond uptime/boot ID, and
mission state. Autonomous missions require sync lock. Recorded frame-read timing
is distinguished from optical exposure time; the latter is not available from the
current polling interface. See the guide for accuracy limits and offline alignment.

```bash
# Record only PX4 telemetry; sends no flight commands.
python3 laptop/telemetry_monitor.py --duration 30 --output /tmp/urex-telemetry.csv
# Or collect telemetry plus native ToF in a new run directory.
python3 laptop/run_experiment.py --mode manual --drone-id 0 --record-tof
```

Use one receiver per UDP port. Do not run the standalone telemetry monitor beside
the experiment runner on the same port. Native frames are queued independently of
the telemetry snapshot rate. The legacy 203-byte 8×8 heatmap stream remains
optional; experiment profiles disable it to avoid duplicate traffic.

## Build

For offline ICP/PGO and a Nav2 occupancy grid from recorded experiment CSVs,
see [the two-stage laptop pipeline](laptop/OFFLINE_SLAM.md).

The latest hardware run used ESP-IDF 6.0.2 and a Seeed Studio XIAO ESP32S3 Sense.

```bash
git clone --recurse-submodules <repository-url>
cd nus_flight_app

idf.py set-target esp32s3
idf.py menuconfig
./scripts/idf_logged.sh build
```

Use `scripts/idf_logged.sh` instead of calling `idf.py` directly whenever terminal output must be
kept. Each invocation prints and saves the exact command, timestamp, Git revision,
output, and exit status. A sibling `.evidence` directory preserves the full commit ID, tracked
binary patch, untracked files, `sdkconfig`, defaults, partition/dependency inputs, and hashes of the
resulting ELF and binaries. This makes dirty development images reproducible without silently
committing unrelated work. Routine logs and evidence go into the ignored `run-logs/` directory so
they do not accidentally bloat Git history. Evidence can contain Wi-Fi credentials from
`sdkconfig`; keep it private or scrub secrets before sharing. Copy only review-worthy hardware logs
into `experiments/` with descriptive filenames. Historical power-comparison logs
remain in their original subdirectories.

For a hardware run (monitor logs are saved under `run-logs/hardware/`):

```bash
./scripts/idf_logged.sh -p /dev/ttyACM0 flash monitor
```

Under **Drone Configuration**, replace `YOUR_WIFI_SSID`, `YOUR_WIFI_PASSWORD`, and the documentation
address `192.0.2.1` with local values if Wi-Fi telemetry is required. Serial-only bench logging does
not require Wi-Fi.

To give the drone a fixed address, enable **Use a static IPv4 address for the drone**
under **Drone Configuration** in `idf.py menuconfig`. For the current `172.20.10.0/28`
hotspot, the local `sdkconfig` uses:

```ini
CONFIG_WIFI_STATIC_IP=y
CONFIG_DRONE_STATIC_IPV4_ADDR="172.20.10.8"
CONFIG_WIFI_STATIC_GATEWAY="172.20.10.1"
CONFIG_WIFI_STATIC_NETMASK="255.255.255.240"
CONFIG_HOST_IPV4_ADDR="172.20.10.3"
```

The first address belongs to the drone; `CONFIG_HOST_IPV4_ADDR` belongs to the laptop
receiving its telemetry. Choose a unique drone address and reserve it or exclude it
from the hotspot's DHCP pool to avoid collisions. Match the gateway and subnet mask
to the actual network when switching hotspots. Disable `CONFIG_WIFI_STATIC_IP` to
restore DHCP. The UDP endpoints use numeric IPv4 addresses, so they do not need DNS.
Rebuild and flash after changing these settings.

Static addressing bypasses DHCP; the SSID, password, and a compatible 2.4 GHz access
point are still required. Wi-Fi disconnect logs include the numeric ESP-IDF reason
and a hint (for example, 201 means the access point was not found, and 202 means
authentication failed). An association message followed by no `Got IP` message
indicates that the link connected but IPv4 is not ready.

The review configuration keeps **Enable AprilTag camera task** disabled. MAVLink, ToF, Wi-Fi, and
NanoSLAM run on Core 0; no project task is started on Core 1 in bench mode. The camera code is
retained and both its detector and driver are assigned to Core 1 when the option is enabled later.

## Onboard NanoSLAM and the laptop map

While armed, the drone records a pose and a 400 ms ToF scan every 1.5 s (64 poses = 96 s),
matches scans with ICP (consecutive scans and loop closures) and solves the pose graph with the
ETH sparse least-squares solver every 5 poses. After disarming it runs a final optimisation and
uploads the graph over Wi-Fi (UDP `CONFIG_NANOSLAM_UPLOAD_PORT`, default 5008). The laptop builds
the map:

```bash
python3 laptop/nanoslam_receive.py            # leave running, then land and disarm
```

Each run is written to `results/slam/<profile>_run<N>_<date>_<time>/` (e.g.
`8x8_15hz_run3_20261108_154646`), the same name as that flight's `results/experiments/` recording: `overview.png` (raw vs optimised
trajectory, points, ICP constraints), `map/` (Nav2 `map.pgm` + `map.yaml`, optimised poses),
`map-raw/` (same with PX4 odometry, for comparison), `slam_result.npz`, CSVs, and
`packets.bin` (`--replay` rebuilds a run). To fly it, prepare a build with
`scripts/prepare_experiment.py --stage <stage> --profile 8x8_15 --slam`.

Host tests (no hardware):

```bash
cmake -S components/nanoslam -B /tmp/nanoslam-build
cmake --build /tmp/nanoslam-build
ctest --test-dir /tmp/nanoslam-build --output-on-failure
python3 -m pytest -q laptop/tests/test_nanoslam_receive.py
```

## Live ToF sensor heatmaps

The laptop viewer shows **all eight raw 8×8 grids** at once, with a shared distance
scale: **red is near, blue is far**. Every cell shows metres; `*` marks target
status 9 (weak signal). Invalid status/range pixels are grey. Frames are labelled
individually as fresh (up to 250 ms), recent (up to 1500 ms, dimmed), stale, offline,
or waiting. Stale/offline grids are cleared to grey rather than displaying old
obstacles as current measurements.

From the `nus_flight_app` directory:

```bash
python3 -m pip install matplotlib numpy
python3 laptop/tof_heatmap.py
```

The viewer listens on **UDP port 5007**. Set **Drone Configuration → Wifi SSID**,
**Wifi Password**, and **Laptop telemetry destination IPv4 address** (`CONFIG_HOST_IPV4_ADDR`, the laptop's Wi-Fi
IPv4 address), then build and flash the updated firmware using the logged workflow
above. The laptop and ESP32 must be on a network that allows them to reach one
another, and the laptop must accept inbound UDP 5007. USB is used for flashing;
this viewer receives the sensor frames over Wi-Fi. Close `tof_debug.py` before
starting the viewer because both listen on the same port.

The updated firmware sends one packet per sensor per 100 ms (80 packets/s for the
ring). **Reflash to see all eight sensors**: the older firmware sends only the front
sensor, leaving seven panels waiting. Packets preserve the original 199-byte prefix
and append a little-endian uint32 sender uptime, for 203 bytes total. This allows
the viewer to include the frame's age on the ESP32 before receipt. Repeated packets
with the same acquisition timestamp do not refresh the reading's age. The older
199-byte packets are accepted, but their initial age on the ESP32 is unavailable.
Reported ages include source age and host time since receipt; network transit time
cannot be measured from these unsynchronized clocks.

The viewer locks to the first valid sender to avoid combining two drones' sensors.
Specify a sender explicitly, change freshness thresholds, or preview without hardware:

```bash
python3 laptop/tof_heatmap.py --source 192.168.1.42 --fresh-ms 250 --recent-ms 1000
python3 laptop/tof_heatmap.py --demo
python3 laptop/tof_heatmap.py --demo --save /tmp/tof-preview.png
python3 -m unittest discover -s laptop -p test_tof_heatmap.py -v
```

Sensor labels use the mux channel and bearing clockwise from the nose. The front
index defaults to this checkout's `sdkconfig` (or Kconfig's default 0 if absent);
override it with `--front-idx 1` to match another board. Grids preserve raw row/column
order; individual sensor mounting rotation and lens mirroring are not corrected.
Hover over a cell to see its raw millimetres, target status, and frame age in the
Matplotlib status bar. The diagnostic range defaults to 50–4000 mm (`--min-mm` and
`--max-mm`); the flight navigation code separately gates ranges at 3000 mm.
Freshness labels describe sensor data, independently of valid target count.

Implementation evidence: target-status gating follows `main/tof_task.c` (statuses
5 and 9). The project paper-index query also found NanoSLAM's separate pixel
validity and distance matrices in [`papers/2309.12008.pdf`, PDF page 10](../../papers/2309.12008.pdf#page=10).
This is research context, not a competition requirement. The index reported
`missing-or-stale`; it was queried as-is without regenerating artifacts.

## Latest bench logs

The preserved logs are indexed in [`experiments/README.md`](experiments/README.md#preserved-log-index).
Power conditions are explicit in the directory names:

- `experiments/with-external-tof-power/`
- `experiments/without-external-tof-power/`

The logs are evidence from props-off development runs, including failures. They are not proof that
the firmware is ready for flight.

## Review focus

The most relevant project changes are the NanoSLAM bridge and upload in `main/slam_bridge.c`, ToF
acquisition and recovery in `main/tof_task.c`, PX4 pose handling (including jump-free SLAM odometry)
in `main/mavlink_task.c`, and the SLAM implementation in `components/nanoslam/`. See `UREX_INTEGRATION.md` for the current
status and `experiments/README.md` for reproduction steps and interpretation guidance.
