# Offline ICP, pose-graph optimization and Nav2 occupancy maps

Use the `telemetry.csv` and `tof.csv` from **experiment 3 or 4**. Experiments 1–2
disable the ToFs, so they cannot produce a range-based map. These programs run on
the laptop, open no network sockets and send no flight commands. No ESP32-S3
pose-graph computation is needed.

## Run the two stages

Dependencies (inside your Python environment):

```bash
python3 -m pip install -r laptop/requirements-offline.txt
```

Set the actual run directory, containing both CSVs:

```bash
RUN=results/experiments/<recorded-run>
```

Review and edit `config/experiments/tof_geometry.example.json` first, or supply
your own geometry JSON. Then run:

```bash
python3 laptop/offline_slam.py "$RUN" \
  --geometry config/experiments/tof_geometry.example.json

python3 laptop/make_occupancy_grid.py "$RUN/slam/slam_result.npz" \
  --output "$RUN/nav2-map" --resolution 0.05 --preview
```

Both output directories must be new. For a repeat with different parameters, use
`offline_slam.py --output "$RUN/slam-repeat"` and a new map output directory.
Neither script overwrites recordings or an earlier result. Each SLAM invocation
writes a new `aligned.csv` inside its output directory, so it cannot silently
reuse stale alignment from an earlier analysis.

The second script writes:

| File | Contents |
| --- | --- |
| `map.pgm` | Binary P5 grayscale: black occupied (0), white free (254), grey unknown (205) |
| `map.yaml` | Relative image filename, metres/cell, origin, `negate: 0`, `mode: trinary`, thresholds |
| `occupancy.npz` | Floating occupancy probabilities, observed-cell mask, log odds and metric origin |
| `summary.json` | Map size, cell counts, geometry and source SLAM status |
| `map.png` | Same raster, when `--preview` is supplied |

Open `map.pgm` in an image viewer, or configure Nav2's map server with the
**absolute path to `map.yaml`**. File-format compatibility does not establish
that the map is accurate enough for navigation. No ROS installation is needed
to generate these files. Output follows the [Nav2 map I/O implementation](https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/src/map_io.cpp):
the image starts with the north/top row; YAML origin specifies the southwest
corner, and the image filename is relative to the YAML file.

To compare against the original PX4 trajectory:

```bash
python3 laptop/make_occupancy_grid.py "$RUN/slam/slam_result.npz" \
  --output "$RUN/nav2-map-raw" --raw --resolution 0.05 --preview
```

The raw/corrected maps can have different metric origins and extents; compare
them in world coordinates using their YAML origins, not by aligning image edges.

## Geometry and the 2D assumption

The example is **uncalibrated**. It uses the current local configuration's front
sensor index **1**, the project's counterclockwise ring numbering, a nominal
45-degree horizontal field of view, radial distance interpretation and zero
mounting offsets. These are starting assumptions, not measurements. Geometry is
not recoverable from `telemetry.csv` and `tof.csv` alone. In particular, record
which physical sensor is connected to each mux channel. Do not mark the geometry
calibrated until it has been checked against measured wall placements.

JSON settings:

- `sensor_yaw_deg_cw`: eight headings clockwise from the drone nose in body FRD.
- `sensor_xy_body_frd_m`: eight `[forward, right]` sensor offsets from the PX4
  pose origin, in metres. Include the actual estimator/body-reference offset.
- `horizontal_fov_deg`: horizontal field of view used for equally spaced column
  bearings. This model does not represent calibrated per-zone lens distortion.
- `column_zero_clockwise`: true when column zero lies clockwise/right of the
  sensor centre. Verify zone orientation on the actual mounted ring.
- `distance_model`: `radial` treats distance as ray length; `axial` treats it as
  forward depth along the sensor's centre axis. Verify the applicable sensor
  distance convention rather than selecting whichever makes a nicer map.
- `calibrated`: explicit true/false, retained in all output metadata.

The projection uses the nearest valid return from the **central two rows of
each column**, for native 4×4 or 8×8 data. Status 5/9 and the configured distance
interval are accepted. Invalid/no-target/clipped returns are omitted; they do
not clear free space. Large roll/pitch is rejected (default body tilt >10°).
This is a planar horizontal approximation for near-level flight among vertical
walls. It does not reconstruct 3D surfaces, model beam width or correct for
floor/ceiling intersections, sensor mounting pitch/roll or terrain. A finer map
cell size does not create additional sensor accuracy.

Coordinates are converted from PX4 NED/body FRD into local ENU/body FLU:
`x = East`, `y = North`, yaw counterclockwise from East. The map frame has the
PX4 local origin; it is not a georeferenced/global ENU frame. Per-frame quaternion
yaw and position are interpolated by the existing alignment code.

## What the SLAM stage does

1. Check one drone, one ESP boot and no PX4 clock reset. Split a recording at
   resets before processing; it is rejected rather than joined into one graph.
2. Align ToF references to bracketing PX4 position and attitude samples, with
   clock lock, readiness uncertainty and gap checks. Reject invalid alignments.
3. Form 0.5-second scan windows by default. Each individual sensor frame is
   transformed using its own interpolated pose into the window's first pose.
   This compensates motion using PX4 odometry; the eight sensors are not treated
   as simultaneously sampled. A window must include validly aligned frames
   from all eight sensors and at least 12 projected points by default.
4. Add PX4 relative-odometry edges between scan poses. Match adjacent scans with
   point-to-line ICP: voxel downsampling, nearest-neighbour correspondences,
   local wall normals, trimming and observability checks. Accepted matches add
   additional graph edges. The 0.45 m normal-neighbourhood and other thresholds
   are starting values and may reject sparse 4×4 scans.
5. Try revisits near the **uncorrected** trajectory, separated by at least ten
   nodes and 2 m travelled by default. Gate overlap, normal residual, correction
   size, convergence and forward/reverse consistency. At most one loop edge is
   added per new node. Similar-looking walls may still produce wrong matches;
   inspect accepted edges. Large drift outside the candidate radius/correction
   limits can prevent closure. A commanded rectangle does not force closure.
6. Optimize SE(2) poses with a fixed first node, sparse least squares and a
   robust loss. All graph noise weights are configurable **heuristics**, not
   calibrated covariances. ICP uses the same odometry to initialize/de-skew the
   scans, so treating the factors as independent also approximates correlation.
7. Save corrected/raw poses and local rays. The occupancy stage reprojects every
   retained scan at its corrected pose, then raycasts from actual sensor origins
   with bounded log-odds updates. Hits take priority over frees in each scan;
   each scan contributes at most one hit/miss per cell to reduce repeated-zone
   confidence inflation. Unobserved and ambiguous cells remain unknown.

Within-window motion compensation remains based on raw odometry; graph
optimization applies a rigid correction to the whole window. Shorter windows
reduce this approximation but collect fewer points. The per-frame reference is
data-ready/readout time, **not known optical exposure time**. Software alignment
cannot recover missing measurements or eliminate unknown sensor latency.

ICP diagnostics are in `slam/icp_diagnostics.json`. Graph constraints and
trajectories are in `graph_edges.csv` and `trajectory.csv`; `summary.json`
contains rejection counts, input hashes, parameters, graph edge counts and
optimizer status. If no ICP passes, the output says `odometry_only`. If no loop
passes, the script explicitly reports this. A non-converged optimizer does not
export a corrected `slam_result.npz`. Optimizer convergence/residual improvement
does not prove lower physical trajectory error without independent ground truth.

Inspect the diagnostics before relaxing gates. Default alignment limits are
100 ms pose brackets and 10 ms readiness halfwidth. Both can be set explicitly
with `--max-pose-gap-ms` and `--max-readiness-halfwidth-ms`; changing them accepts
greater uncertainty rather than fixing timing. Other options are listed by
`--help`. Processing does not alter firmware settings.

## Verification and references

```bash
python3 -m pytest -q laptop/tests/test_offline_slam.py laptop/tests/test_experiment.py
```

Tests cover known rigid-transform recovery, single-wall rejection, graph
correction with a fixed anchor, 4×4/8×8 zone/axis transforms, clock-reset
rejection, occupancy ray tracing and Nav2 PGM/YAML conventions. A full synthetic
drifting rectangle is aligned, matched, optimized and rendered. This is software
verification; no real-flight map or mapping-accuracy claim is made.

Research context: the project paper index was queried without refreshing its
`missing-or-stale` index. *NanoSLAM*, local PDF `papers/2309.12008.pdf`, page 6,
describes scan construction; pages 10 and 17 describe graph sampling and
scan-matching evaluation. The pinned official NanoSLAM checkout is documented
in `references/README.md`; its GPL-3.0 source is a design reference, not copied
into these Python scripts. This pipeline is an independent planar offline
implementation, not a reproduction of NanoSLAM's full hierarchical algorithm.
