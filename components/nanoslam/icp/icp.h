/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See components/nanoslam/README.md.
 *
 * Adapted from ETH-PBL/NanoSLAM (gap9-app) via Duncan Lau's ESP32-S3 port
 * (SLAM-SAFMC combine_2). Modified for the UREX nus_flight_app.
 */

#ifndef SLAM_FIRMWARE_ICP_H
#define SLAM_FIRMWARE_ICP_H

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Largest cloud icp() accepts (POINTS_IN_SCAN); larger inputs return identity */
#define ICP_MAX_POINTS      320

#define POINTS_SIZEOF(n)    (sizeof(icp_points_t) + (n) * sizeof(icp_point_t))
/* icp()'s static working buffers */
#define ICP_STATIC_BYTES    (3 * POINTS_SIZEOF(ICP_MAX_POINTS) + 2 * ICP_MAX_POINTS * sizeof(int16_t))

#define POINTS_STATIC_ALLOC(name, n) \
uint8_t _##name##_buf[POINTS_SIZEOF(n)]; \
icp_points_t *name = (icp_points_t *) _##name##_buf; \
name->num = (n);

typedef struct {
    float x;
    float y;
} icp_point_t;

typedef struct {
    size_t num;
    icp_point_t items[];
} icp_points_t;

static inline icp_points_t *points_alloc(size_t n) {
    icp_points_t *out = (icp_points_t *) malloc(sizeof(icp_points_t) + n * sizeof(icp_point_t));
    if (out != NULL) {
        out->num = n;
    }
    return out;
}

static inline void points_copy(icp_points_t *out, const icp_points_t *in) {
    memcpy(out, in, sizeof(icp_points_t) + in->num * sizeof(icp_point_t));
}

static inline float point_norm(icp_point_t p) {
    return sqrtf(p.x * p.x + p.y * p.y);
}

static inline void point_normalize(icp_point_t *p) {
    float norm = point_norm(*p);
    if (norm > 1e-6f) {
        p->x /= norm;
        p->y /= norm;
    }
}

static inline float points_dist_squared(icp_point_t a, icp_point_t b) {
    float x = a.x - b.x;
    float y = a.y - b.y;
    return x * x + y * y;
}

void svd(const float in[4], float u[4], float sigma[4], float v[4]);

void icp(const icp_points_t *Q, const icp_points_t *P, int max_iterations,
         float error_bound, icp_point_t *translation, float *rotation);

#endif // SLAM_FIRMWARE_ICP_H
