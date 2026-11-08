/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See components/nanoslam/README.md.
 *
 * Graph-based LS-SLAM front end (ICP constraints + ls_slam), adapted from
 * ETH-PBL/NanoSLAM via Duncan Lau's ESP32-S3 port.
 */

#ifndef SLAM_H
#define SLAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "graph-based-slam.h"
#include "slam_graph.h"

/* 63 sequential + up to 65 loop closures for a full 64-pose graph. 64 was
 * hit exactly on the 2026-11-08 43-pose flight, cutting off loop closures. */
#define SLAM_MAX_CONSTRAINTS 128

typedef enum {
    SLAM_ICP_POINT_TO_POINT = 0, /* ETH / Duncan */
    SLAM_ICP_POINT_TO_LINE = 1,  /* as laptop/offline_slam.py (default) */
} slam_icp_method_t;

void slam_set_icp_method(slam_icp_method_t method);

/* Edge weights: per-axis sigmas for odometry (from the raw odometry poses)
 * and ICP constraints. SLAM_WEIGHTS_ETH reproduces ETH/Duncan's weighting. */
#define SLAM_WEIGHTS_DEFAULT {.odom_sigma_xy = 0.08f, .odom_sigma_yaw = 0.01745f, \
                              .icp_sigma_xy = 0.03f, .icp_sigma_yaw = 0.05236f}
#define SLAM_WEIGHTS_ETH     {.odom_sigma_xy = 1.0f, .odom_sigma_yaw = 1.0f, \
                              .icp_sigma_xy = 0.2236068f, .icp_sigma_yaw = 0.2236068f}
void slam_set_weights(const ls_slam_weights_t *weights);
ls_slam_weights_t slam_get_weights(void);
slam_icp_method_t slam_get_icp_method(void);

/* Point-to-line rejections in the last slam_optimize, indexed by
 * icp_line_status_t; slot 0 (ICP_LINE_ACCEPTED) counts matches rejected for
 * moving the pose too far (> 0.3 m or 15 deg). */
#define SLAM_ICP_REJECT_REASONS 8
void slam_icp_line_rejects(uint16_t out[SLAM_ICP_REJECT_REASONS]);

typedef enum {
    SLAM_CONSTRAINT_SEQUENTIAL = 0, /* consecutive scans */
    SLAM_CONSTRAINT_LOOP = 1,       /* revisit of an earlier pose */
} slam_constraint_kind_t;

typedef struct {
    size_t poses;
    uint16_t sequential_attempts, sequential_accepted;
    uint16_t loop_attempts, loop_accepted;
    bool solved;          /* false: ls_slam rejected the solve, poses unchanged */
    uint32_t icp_ms;      /* time spent building constraints */
    uint32_t solve_ms;    /* time spent in ls_slam */
} slam_report_t;

/* Builds ICP constraints over the current graph and solves it. Writes the
 * optimised poses to poses[0..n-1] and returns n (0 if fewer than 2 poses).
 * Poses recorded while this runs are left alone; the caller re-anchors them.
 * Not re-entrant. report may be NULL. */
size_t slam_optimize(float poses[GRAPH_MAX_POSES][3], slam_report_t *report);

/* Writes poses[0..n-1] back into the graph. */
void slam_commit_poses(const float poses[GRAPH_MAX_POSES][3], size_t n);

/* Static scratch used by slam_optimize (scan copies, point clouds, constraints)
 * plus the ls_slam solver buffers. */
size_t slam_static_bytes(void);

/* Constraints accepted by the most recent slam_optimize, for export. */
size_t slam_last_constraints(const constraint **constraints,
                             const uint8_t **kinds);

#endif /* SLAM_H */
