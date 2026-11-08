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

#include "icp.h"

#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#define H_ZERO  {1, 0, 0, 0, 1, 0, 0, 0, 1}

static void matmul_2x2(const float a[4], const float b[4], float c[4]) {
    c[0] = a[0] * b[0] + a[1] * b[2];
    c[1] = a[0] * b[1] + a[1] * b[3];
    c[2] = a[2] * b[0] + a[3] * b[2];
    c[3] = a[2] * b[1] + a[3] * b[3];
}

static void matmul_3x3(const float a[9], const float b[9], float c[9]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i * 3 + j] = 0;
            for (int k = 0; k < 3; k++) {
                c[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
            }
        }
    }
}

static icp_point_t matpointmul(const float r[4], const icp_point_t p) {
    icp_point_t out = {
            .x = r[0] * p.x + r[1] * p.y,
            .y = r[2] * p.x + r[3] * p.y,
    };
    return out;
}

static void compute_correspondences(const icp_points_t *P, const icp_points_t *Q,
                                    float error_bound, int16_t cor_P[], int16_t cor_Q[]) {
    assert(P->num < INT16_MAX);
    assert(Q->num < INT16_MAX);
    int16_t p_num = (int16_t) P->num;
    int16_t q_num = (int16_t) Q->num;
    memset(cor_Q, 0, Q->num * sizeof(int16_t));
    for (int16_t i = 0; i < p_num; ++i) {
        float min_dist = error_bound;
        int16_t chosen_idx = -1;
        icp_point_t cur_point = P->items[i];
        for (int16_t j = 0; j < q_num; ++j) {
            float dist = points_dist_squared(cur_point, Q->items[j]);
            if (dist < min_dist) {
                min_dist = dist;
                chosen_idx = j;
            }
        }
        cor_P[i] = chosen_idx;
        if (chosen_idx >= 0) cor_Q[chosen_idx]++;
    }
}

static void center_data_P(const icp_points_t *points, const int16_t cor[],
                          icp_point_t *center, icp_points_t *centered) {
    assert(points->num == centered->num);
    float x = 0, y = 0;
    size_t valid_cnt = 0;
    for (size_t i = 0; i < points->num; ++i) {
        if (cor[i] < 0) continue;
        x += points->items[i].x;
        y += points->items[i].y;
        valid_cnt++;
    }
    if (valid_cnt > 0) {
        x /= (float) valid_cnt;
        y /= (float) valid_cnt;
    }
    for (size_t i = 0; i < points->num; ++i) {
        if (cor[i] < 0) continue;
        centered->items[i].x = points->items[i].x - x;
        centered->items[i].y = points->items[i].y - y;
    }
    center->x = x;
    center->y = y;
}

static void center_data_Q(const icp_points_t *points, const int16_t cor[],
                          icp_point_t *center, icp_points_t *centered) {
    assert(points->num == centered->num);
    float x = 0, y = 0;
    size_t num = 0;
    for (size_t i = 0; i < points->num; ++i) {
        if (!cor[i]) continue;
        float count = cor[i];
        x += points->items[i].x * count;
        y += points->items[i].y * count;
        num += cor[i];
    }
    if (num > 0) {
        x /= (float) num;
        y /= (float) num;
    }
    for (size_t i = 0; i < points->num; ++i) {
        if (!cor[i]) continue;
        centered->items[i].x = points->items[i].x - x;
        centered->items[i].y = points->items[i].y - y;
    }
    center->x = x;
    center->y = y;
}

static void compute_cross_variance(icp_points_t *P, icp_points_t *Q,
                                   const int16_t cor[], float cov[4]) {
    memset(cov, 0, sizeof(float) * 4);
    for (size_t i = 0; i < P->num; ++i) {
        int16_t j = cor[i];
        if (j < 0) continue;
        icp_point_t p = P->items[i];
        icp_point_t q = Q->items[j];
        cov[0] += p.x * q.x;
        cov[1] += p.y * q.x;
        cov[2] += p.x * q.y;
        cov[3] += p.y * q.y;
    }
}

void svd(const float in[4], float u[4], float sigma[4], float v[4]) {
    float a = in[0], b = in[1], c = in[2], d = in[3];

    float e = 0.5f * (a + d);
    float f = 0.5f * (a - d);
    float g = 0.5f * (b + c);
    float h = 0.5f * (c - b);

    if (sigma != NULL) {
        float q = sqrtf(e * e + h * h);
        float r = sqrtf(f * f + g * g);

        float sx = q + r;
        float sy = q - r;

        sigma[0] = sx;
        sigma[1] = 0;
        sigma[2] = 0;
        sigma[3] = sy;
    }

    float a1 = atan2f(g, f);
    float a2 = atan2f(h, e);

    float theta = 0.5f * (a2 - a1);
    float phi = 0.5f * (a2 + a1);

    float theta_sin = sinf(theta);
    float theta_cos = cosf(theta);
    float phi_sin = sinf(phi);
    float phi_cos = cosf(phi);

    u[0] = theta_cos;
    u[1] = -theta_sin;
    u[2] = theta_sin;
    u[3] = theta_cos;

    v[0] = phi_cos;
    v[1] = -phi_sin;
    v[2] = phi_sin;
    v[3] = phi_cos;
}

void icp(const icp_points_t *Q, const icp_points_t *P, int max_iterations,
         float error_bound, icp_point_t *translation, float *rotation) {

    if (P->num == 0 || Q->num == 0 || P->num > ICP_MAX_POINTS || Q->num > ICP_MAX_POINTS) {
        translation->x = 0;
        translation->y = 0;
        *rotation = 0;
        return;
    }

    /* Working buffers are static, as in ETH's original (cloud0/1/2): Duncan's
     * stack arrays need ~9 KB and overflowed the 8 KB in-flight optimiser
     * stack on 2026-11-07, rebooting the ESP32 mid-flight. Not re-entrant;
     * only the SLAM optimiser calls icp(). */
    static uint8_t p_copy_buf[POINTS_SIZEOF(ICP_MAX_POINTS)] __attribute__((aligned(4)));
    static uint8_t p_centered_buf[POINTS_SIZEOF(ICP_MAX_POINTS)] __attribute__((aligned(4)));
    static uint8_t q_centered_buf[POINTS_SIZEOF(ICP_MAX_POINTS)] __attribute__((aligned(4)));
    static int16_t cor_P[ICP_MAX_POINTS];
    static int16_t cor_Q[ICP_MAX_POINTS];
    icp_points_t *P_copy = (icp_points_t *)p_copy_buf;
    icp_points_t *P_centered = (icp_points_t *)p_centered_buf;
    icp_points_t *Q_centered = (icp_points_t *)q_centered_buf;
    points_copy(P_copy, P);
    P_centered->num = P->num;
    Q_centered->num = Q->num;

    float t_a[9] = H_ZERO, t_b[9] = H_ZERO;
    float *next = t_a, *prev = t_b;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {

        // Compute correspondences
        compute_correspondences(P_copy, Q, error_bound, cor_P, cor_Q);

        // Center the data for P and Q (based on correspondences)
        icp_point_t center_of_P, center_of_Q;
        center_data_P(P_copy, cor_P, &center_of_P, P_centered);
        center_data_Q(Q, cor_Q, &center_of_Q, Q_centered);

        // Compute the cross variance on centered data
        float cov[4];
        compute_cross_variance(P_centered, Q_centered, cor_P, cov);

        // Execute SVD to obtain transformation
        float u[4], sigma[4], v[4], r[4];
        svd(cov, u, sigma, v);
        matmul_2x2(u, v, r);
        icp_point_t t = matpointmul(r, center_of_P);
        t.x = center_of_Q.x - t.x;
        t.y = center_of_Q.y - t.y;

        // Chain transformations using homogeneous coordinates
        float transform[9] = {
                r[0], r[1], t.x,
                r[2], r[3], t.y,
                0, 0, 1,
        };
        matmul_3x3(transform, prev, next);
        float *tmp = next;
        next = prev;
        prev = tmp;

        // Apply transformation to point cloud
        for (size_t i = 0; i < P_copy->num; ++i) {
            icp_point_t p = matpointmul(r, P_copy->items[i]);
            p.x += t.x;
            p.y += t.y;
            P_copy->items[i] = p;
        }
    }

    // Write transform result
    translation->x = prev[2];
    translation->y = prev[5];
    *rotation = atan2f(prev[3], prev[0]);
}
