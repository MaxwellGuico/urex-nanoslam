# ToF integration time: bench test and flight check (2026-11-08)

**Result: use 10 ms integration for the 8x8 / 15 Hz profile** (`CONFIG_TOF_INTEGRATION_MS=10`). It
returns about a third more wall points per second than the previous 5 ms at the same frame rate.
20 ms and 40 ms give better individual frames but slow the sensors to 7.9 Hz and 3.0 Hz, which
loses data overall and would make SLAM drop frames older than 100 ms.

## Why

On the first 4 x 5.5 m flight (`8x8_15hz_run7_20261108_163218`, 5 ms) only 29% of the centre-row
zones returned a target (68% "no target", status 255), against 68% in the 3.2 x 3.4 m room. Valid
readings rarely exceeded 2.4 m, so most scans saw too few walls for ICP: 27 of 63 consecutive
matches and 6 of 31 loop closures were accepted. ST's default integration time is 20 ms; the
firmware used 5 ms.

## Bench test

- Drone 8, props on, battery in, bench-mode firmware (cannot arm or publish setpoints; RC off,
  QGC arm not used), raised to about flight height on a stand in the big space, same position
  and heading for every recording, 20 s each.
- Builds: `build-experiments/manual-8x8_15-int{5,10,20,40}ms`
  (`scripts/prepare_experiment.py --stage manual --profile 8x8_15 --tof-integration-ms N --name intNms`).
- Recording: `laptop/run_experiment.py --mode manual --record-tof --profile 8x8_15 --drone-id 8 --duration 20`.
  The integration time is reported in telemetry (`tof_integration_ms` column), so each recording
  proves its own setting.
- Analysis: `laptop/tof_signal_report.py <runs> --plot results/previews/tof_integration_20261108.png`.
- Caveat: for the 20 ms and 40 ms recordings, obstacles were placed in front of and behind the
  drone at about 0.5 m. Those returns are excluded below by counting only targets beyond 1 m; they
  also hid part of the far front/back walls, so 20/40 ms are slightly understated.

| Integration | Recording | Frame rate per sensor | Valid zones per frame | Wall returns > 1 m per second | > 2 m per second | Farthest |
| --- | --- | --- | --- | --- | --- | --- |
| 5 ms | `8x8_15hz_run10_20261108_170951` | 14.2 Hz | 21% | 447 | 103 | 3.06 m |
| **10 ms** | `8x8_15hz_run12_20261108_171307` | **14.2 Hz** | 30% | **587 (+31%)** | **173 (+68%)** | 3.53 m |
| 20 ms | `8x8_15hz_run13_20261108_171524` | 7.9 Hz | 53%* | 391 | 128 | 3.09 m |
| 40 ms | `8x8_15hz_run14_20261108_171718` | 3.0 Hz | 59%* | 167 | 58 | 3.56 m |

\* includes the placed obstacles.

Up to 10 ms the VL53L5CX keeps the requested 15 Hz in 8x8 mode; beyond that the frame period grows
with integration time. The point where the rate starts to fall between 10 and 20 ms was not
measured (a 15 ms recording would show it). The 4x4 / 30 Hz profile (limit 27 ms) was not tested.

## Flight check

Same 4 x 5.5 m mission from the middle of the bottom edge, 8x8 / 15 Hz, point-to-line ICP, 2 s
SLAM pose interval:

| | Run 7 (5 ms) | Run 15 (10 ms, `8x8_15hz_run15_20261108_172314`) |
| --- | --- | --- |
| Zones with a target | 29% | 36% |
| 95% of valid readings under | 2.44 m | 2.85 m |
| Usable columns per scan (median, max 320) | 94 | 112 |
| Consecutive ICP matches accepted | 27 / 63 | 30 / 63 |
| Loop closures accepted | 6 / 31 | 12 / 28 |

The improvement is real but moderate: most of the big space is still beyond what the sensors
return. The same flight showed the solver weighting problem documented in
[`components/nanoslam/README.md`](../components/nanoslam/README.md#differences-from-eth-nanoslam-and-duncans-port)
(maps rotated 6-7 deg away from PX4's stable heading).
