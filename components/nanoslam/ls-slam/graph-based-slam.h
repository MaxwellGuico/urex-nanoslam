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

#ifndef GRAPHBASEDSLAM_H
#define GRAPHBASEDSLAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "sparse-matrix.h"

typedef struct {
    int16_t fromIdx;
    int16_t toIdx;
    float z[3];
} constraint;

typedef struct {
    float z[3];
} measurement;

void linearize_pose_pose_constraint(float *x1, float *x2, float *z, float A[3][3], float B[3][3], float *e);

void build_H_and_b(float *x,
                   sparseMatrix *H,
                   float *b,
                   measurement *measurements,
                   const uint8_t *skip,
                   constraint *constraints,
                   int16_t nr_of_poses,
                   int16_t nr_of_constr);

bool solve_for_x(sparseMatrix *H, float *b, int16_t *perm, int16_t size);

void measurement_from_poses(float *x_from, float *x_to, float *z);

void measurements_from_poses(float *x, measurement *measurements, int nr_of_poses);

void forward_substitution(sparseMatrix *L, float *b, int N);

void backward_substitution(sparseMatrix *L, float *b, int N);

/* Three Gauss-Newton iterations over the pose graph (x = [x y yaw] per pose,
 * pose 0 anchored). Odometry edges come from the current x; constraints[i]
 * must have fromIdx > toIdx (newer -> older), as build_H_and_b stores the
 * off-diagonal blocks in the lower triangle. Returns false, leaving x
 * unchanged, if the sparse matrices overflow or the update is not finite. */
size_t ls_slam_static_bytes(void);
size_t ls_slam_heap_bytes(void);

bool ls_slam(float *x, uint8_t *skip, constraint *constraints, int16_t nr_of_poses, int16_t nr_of_constr);

/* As ls_slam, but odometry[i] (i = 0 .. n-2) is the measured motion from pose
 * i to i+1 instead of the motion between the current estimates in x. ETH
 * derives odometry from x, which is fine for a single solve but, when the
 * result is committed and solved again (in flight), re-counts every earlier
 * correction as if it had been measured. Pass the raw odometry to avoid that.
 * odometry may be NULL (= ETH behaviour). */
bool ls_slam_odometry(float *x, const measurement *odometry, uint8_t *skip,
                      constraint *constraints, int16_t nr_of_poses, int16_t nr_of_constr);

typedef struct {
    float odom_sigma_xy, odom_sigma_yaw; /* m, rad */
    float icp_sigma_xy, icp_sigma_yaw;   /* m, rad */
} ls_slam_weights_t;

/* Per-axis standard deviations for odometry and constraint edges (x and y
 * share one value). Default: ETH's unit odometry / 20x isotropic ICP. */
void ls_slam_set_weights(const ls_slam_weights_t *weights);

float test_linear_system(sparseMatrix *H, float *x, float *b, int N);

#endif // GRAPHBASEDSLAM_H
