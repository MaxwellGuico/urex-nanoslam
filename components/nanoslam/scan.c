/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See components/nanoslam/README.md.
 *
 * Adapted from ETH-PBL/NanoSLAM via Duncan Lau's ESP32-S3 port
 * (SLAM-SAFMC combine_2). Modified for the UREX nus_flight_app.
 */

#include "scan.h"

#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_4
#define M_PI_4 (M_PI / 4.0)
#endif

static float s_sensor_yaw[SCAN_NUM_DIRS];
static float s_sensor_offset_m = 0.025f;
static bool s_geometry_set = false;
static int s_cols = SCAN_NUM_COLS;

void scan_set_columns(int columns) {
    s_cols = (columns == 4) ? 4 : SCAN_NUM_COLS;
}

int scan_get_columns(void) { return s_cols; }

void scan_set_geometry(const float sensor_yaw_rad[SCAN_NUM_DIRS], float sensor_offset_m) {
    for (int k = 0; k < SCAN_NUM_DIRS; k++) {
        s_sensor_yaw[k] = sensor_yaw_rad[k];
    }
    s_sensor_offset_m = sensor_offset_m;
    s_geometry_set = true;
}

static float sensor_yaw(int dir) {
    if (s_geometry_set) return s_sensor_yaw[dir];
    /* Duncan's layout: sensors counter-clockwise, sensor 0 at the nose. */
    return -(float)dir * (2.0f * (float)M_PI / SCAN_NUM_DIRS);
}

void scan_extract_points(const scan_t *scan, float x, float y, float yaw,
                         icp_points_t *points) {
    const float step = -((float)M_PI_4) / (float)s_cols;

    points->num = 0;

    for (int frame = 0; frame < SCAN_NUM_FRAMES; frame++) {

        float frame_x = x + scan->frames[frame].dx;
        float frame_y = y + scan->frames[frame].dy;
        float frame_yaw = yaw + scan->frames[frame].dyaw;

        for (int dir = 0; dir < SCAN_NUM_DIRS; dir++) {

            /* NED yaw is clockwise-positive */
            float dir_yaw = frame_yaw + sensor_yaw(dir);

            /* Column 0 is the most clockwise column (+19.7 deg for 8x8,
             * +16.9 deg for 4x4) */
            float angle = -(float)(s_cols / 2) * step + step / 2.0f;
            for (int col = 0; col < s_cols; angle += step, col++) {

                int16_t measurement = scan->frames[frame].dists[dir][col];
                if (measurement < SCAN_MIN_VALID_MM || measurement > SCAN_MAX_VALID_MM)
                    continue;

                /* Radial ToF beam projection */
                float range = (float)measurement / 1000.0f;
                float dist_x = range * cosf(angle) + s_sensor_offset_m;
                float dist_y = range * sinf(angle);

                icp_point_t p = {
                    .x = frame_x + dist_x * cosf(dir_yaw) - dist_y * sinf(dir_yaw),
                    .y = frame_y + dist_x * sinf(dir_yaw) + dist_y * cosf(dir_yaw),
                };
                points->items[points->num++] = p;
            }
        }
    }
}

static int compare_int16(const void *a, const void *b) {
    return (*(const int16_t *)a - *(const int16_t *)b);
}

int scan_reduce_frame(const int16_t distance_mm[SCAN_ZONES],
                      const uint8_t target_status[SCAN_ZONES],
                      int side,
                      int16_t measurements[SCAN_NUM_COLS]) {
    if (side != 4) side = 8;
    int rejected = 0;
    scan_invalidate_dists(measurements);
    for (int col = 0; col < side; col++) {
        int16_t rows[4];
        uint8_t num_rows = 0;
        /* Centre rows (~+-11 deg about horizontal): rows 2-5 of 8x8 as in the
         * ETH original, rows 1-2 of 4x4. The outer rows hit the floor/ceiling
         * at cruise altitude and produced rings of fake obstacles. */
        for (int row = side / 4; row < 3 * side / 4; row++) {
            int idx = row * side + col;
            uint8_t status = target_status[idx];
            int16_t dist = distance_mm[idx];
            /* Status 5 = valid, 9 = valid with weak signal. */
            if ((status == 5 || status == 9) && dist >= SCAN_MIN_VALID_MM &&
                dist <= SCAN_MAX_VALID_MM) {
                rows[num_rows++] = dist;
            }
        }

        if (num_rows == 0) {
            measurements[col] = -1;
            continue;
        }

        qsort(rows, num_rows, sizeof(int16_t), compare_int16);
        if (num_rows < SCAN_MIN_ROWS ||
            rows[num_rows - 1] - rows[0] > SCAN_MAX_ROW_SPREAD_MM) {
            measurements[col] = -1;
            rejected++;
            continue;
        }
        if (num_rows & 1) {
            measurements[col] = rows[num_rows >> 1];
        } else {
            measurements[col] =
                (int16_t)((rows[num_rows >> 1] + rows[(num_rows >> 1) - 1]) >> 1);
        }
    }
    return rejected;
}

void scan_invalidate_dists(int16_t dists[SCAN_NUM_COLS]) {
    for (int j = 0; j < SCAN_NUM_COLS; j++) {
        dists[j] = -1;
    }
}

int scan_count_valid(const scan_t *scan) {
    int n = 0;
    for (int i = 0; i < SCAN_NUM_FRAMES; i++)
        for (int dir = 0; dir < SCAN_NUM_DIRS; dir++)
            for (int col = 0; col < SCAN_NUM_COLS; col++)
                if (scan->frames[i].dists[dir][col] >= 0) n++;
    return n;
}
