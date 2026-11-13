/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See components/nanoslam/README.md.
 *
 * Adapted from ETH-PBL/NanoSLAM via Duncan Lau's ESP32-S3 port
 * (SLAM-SAFMC combine_2). Modified for the UREX nus_flight_app: the
 * acquisition loop lives in main/slam_bridge.c so this file has no hardware
 * dependencies, and sensor headings come from a table instead of a fixed
 * front-index formula.
 */

#ifndef SCAN_H
#define SCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "icp.h"

/* Every ToF sensor is used (ETH used 4 and rotated the drone 45 deg per scan
 * to fill the gaps). dists[k] holds physical sensor k. SCAN_NUM_COLS is the
 * storage size; a 4x4 profile uses the first 4 columns (scan_set_columns). */
#define SCAN_NUM_DIRS   8
#define SCAN_NUM_COLS   8
#define SCAN_ZONES      64

#define SCAN_NUM_FRAMES 5
#define POINTS_IN_SCAN  (SCAN_NUM_FRAMES * SCAN_NUM_DIRS * SCAN_NUM_COLS)
_Static_assert(POINTS_IN_SCAN <= ICP_MAX_POINTS, "icp() buffers must hold a full scan");
#define SCAN_TIME_MS    400
/* ~20% of Duncan's logged points were at 0.2-0.4 m (prop guards / chassis). */
#define SCAN_MIN_VALID_MM 300
/* Beams are ~30 cm wide at 3 m; far readings were the noisiest. */
#define SCAN_MAX_VALID_MM 2500
/* Drop a sensor frame measured this long before the pose it is attached to */
#define SCAN_MAX_FRAME_AGE_MS 100
/* ...or if the drone turned more than this (~3 deg, half a column) since then */
#define SCAN_MAX_YAW_ERR_RAD 0.05f
/* Centre rows that must be valid and agree for a column to count, which
 * rejects mixed pixels where a beam straddles an edge */
#define SCAN_MIN_ROWS 2
#define SCAN_MAX_ROW_SPREAD_MM 100
/* Column value when at least SCAN_MIN_ROWS centre rows saw nothing within
 * SCAN_MAX_VALID_MM (status 255 "no target", or a valid return beyond it) and
 * none saw a hit. The laptop map clears free space along it; ICP skips it. */
#define SCAN_NO_TARGET (-2)

typedef struct {
    int16_t dists[SCAN_NUM_DIRS][SCAN_NUM_COLS]; /* mm; -1 = no measurement, SCAN_NO_TARGET = nothing in range */
    float dx, dy, dyaw; /* drone motion since the scan's first frame */
} scan_frame_t;

typedef struct scan {
    uint16_t pose;      /* graph node index */
    scan_frame_t frames[SCAN_NUM_FRAMES];
} scan_t;

/* Body-frame heading of each sensor (radians, clockwise from the nose, NED
 * convention) and the lens distance from the drone centre. Call once before
 * extracting points; defaults to sensor k at -k*45 deg and 0.025 m. */
void scan_set_geometry(const float sensor_yaw_rad[SCAN_NUM_DIRS], float sensor_offset_m);

/* Columns per sensor: 8 for the 8x8 profile (default), 4 for 4x4. The 45 deg
 * field of view is split evenly between them. */
void scan_set_columns(int columns);
int scan_get_columns(void);

/* Reduce one VL53L5CX frame (side x side zones, row-major, side 8 or 4) to a
 * range per column: median of the centre rows covering +-11 deg about
 * horizontal (rows 2-5 of 8x8, rows 1-2 of 4x4). Returns the number of
 * columns rejected because the rows disagreed or too few were valid. Columns
 * beyond `side` are set to -1. */
int scan_reduce_frame(const int16_t distance_mm[SCAN_ZONES],
                      const uint8_t target_status[SCAN_ZONES],
                      int side,
                      int16_t measurements[SCAN_NUM_COLS]);

void scan_invalidate_dists(int16_t dists[SCAN_NUM_COLS]);

/* World-frame (NED x = north, y = east) points of a scan taken at pose
 * (x, y, yaw). points must hold POINTS_IN_SCAN items. */
void scan_extract_points(const scan_t *scan, float x, float y, float yaw,
                         icp_points_t *points);

/* Number of valid column measurements in a scan. */
int scan_count_valid(const scan_t *scan);

#endif /* SCAN_H */
