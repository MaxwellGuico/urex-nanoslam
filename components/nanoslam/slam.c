/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See components/nanoslam/README.md.
 *
 * Adapted from ETH-PBL/NanoSLAM via Duncan Lau's ESP32-S3 port
 * (SLAM-SAFMC combine_2). Modified for the UREX nus_flight_app: single drone,
 * no FreeRTOS dependency, and a solve that fails is reported instead of
 * silently applied.
 */

#include "slam.h"

#include <math.h>
#include <string.h>

#include "icp.h"
#include "icp_line.h"
#include "nanoslam_port.h"
#include "scan.h"

static const char *TAG = "slam";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* A point counts as an ICP inlier if it lands within this of a ref point. */
#define ICP_INLIER_DIST_M      0.15f
#define ICP_MIN_INLIER_RATIO   0.5f
#define ICP_MIN_POINTS         15
/* Loop closures: only poses at least this many nodes apart, and only the
 * single closest candidate per pose, so one revisit adds one constraint. */
#define LOOP_MIN_NODE_GAP      5
#define LOOP_MAX_DIST_M        1.5f
#define LOOP_MAX_DYAW_RAD      0.8f
/* Max pose correction ICP may apply. Duncan's logged PX4 positions were off
 * by up to 0.9 m (headings by <15 deg); the inlier-ratio check is the real
 * guard. */
#define ICP_MAX_ROT_RAD        0.35f
#define ICP_MAX_TRANS_M        1.0f
#define ICP_ITERATIONS         30
/* Squared correspondence distance passed to icp() (0.6 m^2 = 0.77 m) */
#define ICP_ERROR_BOUND        0.6f

/* Point-to-line gates, as in laptop/offline_slam.py: the correction ICP may
 * apply to the pose (its "innovation") */
#define ICP_LINE_MAX_TRANS_M   0.30f
#define ICP_LINE_MAX_ROT_RAD   0.2618f /* 15 deg */

static slam_icp_method_t s_icp_method = SLAM_ICP_POINT_TO_LINE;

/* Default edge weights (see slam_set_weights). PX4's gyro heading is trusted
 * to 0.1 deg per step, ICP to ~3 deg; ICP position (3 cm) is trusted more than
 * optical-flow odometry (8 cm). Chosen on host replays of the 2026-11-08
 * flights: ETH's 20x isotropic ICP weighting rotated the big-space maps 3-6 deg
 * away from PX4's (stable) heading, and 1 deg per step still let a few bad
 * early loop closures rotate run16 by up to 3.8 deg. */
static ls_slam_weights_t s_weights = SLAM_WEIGHTS_DEFAULT;
static measurement s_odometry[GRAPH_MAX_POSES];
static uint16_t s_line_rejects[ICP_LINE_HIGH_RMSE + 1];

static constraint s_constraints[SLAM_MAX_CONSTRAINTS];
static uint8_t s_kinds[SLAM_MAX_CONSTRAINTS];
static size_t s_num_constraints;

static float wrap_pi(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* Fraction of cur points within ICP_INLIER_DIST_M of a ref point after
 * applying (c, s, t) */
static float inlier_ratio_of(const icp_points_t *ref_scan, const icp_points_t *cur_scan,
                             float c, float s, icp_point_t t) {
    size_t inliers = 0;
    for (size_t k = 0; k < cur_scan->num; k++) {
        icp_point_t q = {
            .x = cur_scan->items[k].x * c - cur_scan->items[k].y * s + t.x,
            .y = cur_scan->items[k].x * s + cur_scan->items[k].y * c + t.y,
        };
        for (size_t r = 0; r < ref_scan->num; r++) {
            if (points_dist_squared(q, ref_scan->items[r]) <
                ICP_INLIER_DIST_M * ICP_INLIER_DIST_M) {
                inliers++;
                break;
            }
        }
    }
    return (float)inliers / (float)cur_scan->num;
}

/* ICP of cur_scan onto ref_scan (both world frame at the current pose
 * estimates). On success writes a constraint cur -> ref (fromIdx > toIdx, as
 * build_H_and_b requires). */
static bool comp_constr(const icp_points_t *ref_scan, const icp_points_t *cur_scan,
                        const float *ref_pose, const float *cur_pose,
                        int16_t ref_pose_idx, int16_t cur_pose_idx,
                        constraint *out_cons) {

    if (ref_scan->num < ICP_MIN_POINTS || cur_scan->num < ICP_MIN_POINTS) {
        return false;
    }

    icp_point_t icp_tr = {0};
    float icp_rot = 0;
    const bool line = s_icp_method == SLAM_ICP_POINT_TO_LINE;
    icp_line_info_t line_info = {0};
    if (line) {
        if (!icp_point_to_line(ref_scan, cur_scan, NULL, &icp_tr, &icp_rot, &line_info)) {
            s_line_rejects[line_info.status]++;
            NS_LOGD(TAG, "ICP %d->%d rejected: %s (overlap %.2f, rmse %.3f, last step %.5f m %.5f rad)",
                    cur_pose_idx, ref_pose_idx, icp_line_status_name(line_info.status),
                    line_info.overlap, line_info.rmse_m, line_info.last_step_m,
                    line_info.last_step_rad);
            return false;
        }
    } else {
        icp(ref_scan, cur_scan, ICP_ITERATIONS, ICP_ERROR_BOUND, &icp_tr, &icp_rot);
    }
    if (!isfinite(icp_tr.x) || !isfinite(icp_tr.y) || !isfinite(icp_rot)) {
        return false;
    }

    float c = cosf(icp_rot);
    float s = sinf(icp_rot);
    float x = cur_pose[0] * c - cur_pose[1] * s + icp_tr.x;
    float y = cur_pose[0] * s + cur_pose[1] * c + icp_tr.y;
    float corrected_pose[3] = {x, y, cur_pose[2] + icp_rot};

    /* Gate on how far ICP moves the drone pose. The raw ICP translation is
     * about the world origin, so it also contains R*pose. */
    float trans_dist = hypotf(x - cur_pose[0], y - cur_pose[1]);
    const float max_rot = line ? ICP_LINE_MAX_ROT_RAD : ICP_MAX_ROT_RAD;
    const float max_trans = line ? ICP_LINE_MAX_TRANS_M : ICP_MAX_TRANS_M;
    if (fabsf(icp_rot) > max_rot || trans_dist > max_trans) {
        if (line) s_line_rejects[0]++; /* slot 0 counts large corrections */
        NS_LOGD(TAG, "ICP %d->%d rejected: rot=%.2f trans=%.2f",
                cur_pose_idx, ref_pose_idx, icp_rot, trans_dist);
        return false;
    }

    /* Reject alignments where too few points land on the reference scan:
     * ICP always converges to something, even on unrelated scans.
     * Point-to-line has already checked overlap and RMSE. */
    float inlier_ratio = line ? line_info.overlap
                              : inlier_ratio_of(ref_scan, cur_scan, c, s, icp_tr);
    if (!line && inlier_ratio < ICP_MIN_INLIER_RATIO) {
        NS_LOGD(TAG, "ICP %d->%d rejected: inlier ratio %.2f",
                cur_pose_idx, ref_pose_idx, inlier_ratio);
        return false;
    }

    out_cons->fromIdx = cur_pose_idx;
    out_cons->toIdx = ref_pose_idx;
    measurement_from_poses(corrected_pose, (float *)ref_pose, out_cons->z);

    NS_LOGI(TAG, "Constraint %d -> %d accepted: z=(%.2f, %.2f, %.2f) "
                 "icp_rot=%.3f trans=%.2f inliers=%.2f",
            cur_pose_idx, ref_pose_idx, out_cons->z[0], out_cons->z[1],
            out_cons->z[2], icp_rot, trans_dist, inlier_ratio);
    return true;
}

/* Scratch, static because slam_optimize is never re-entered */
static scan_t s_ref_scan;
static scan_t s_cur_scan;
static uint8_t s_ref_buf[POINTS_SIZEOF(POINTS_IN_SCAN)] __attribute__((aligned(4)));
static uint8_t s_cur_buf[POINTS_SIZEOF(POINTS_IN_SCAN)] __attribute__((aligned(4)));

static bool try_constraint(float poses[GRAPH_MAX_POSES][3], size_t ref, size_t cur,
                           uint8_t kind) {
    if (s_num_constraints >= SLAM_MAX_CONSTRAINTS) return false;
    if (!slam_graph_get_scan(ref, &s_ref_scan) || !slam_graph_get_scan(cur, &s_cur_scan)) {
        return false;
    }
    icp_points_t *ref_cloud = (icp_points_t *)s_ref_buf;
    icp_points_t *cur_cloud = (icp_points_t *)s_cur_buf;
    /* Run ICP in a frame centred on the reference pose. ICP rotates about the
     * origin, so with points far from it (PX4's origin drifts with removed
     * resets: ~77 m on 2026-11-13 run 2) rotation and translation become
     * indistinguishable and point-to-line rejected every match. The
     * constraint is relative, so the shift does not change it. */
    const float ref_local[3] = {0.0f, 0.0f, poses[ref][2]};
    const float cur_local[3] = {poses[cur][0] - poses[ref][0], poses[cur][1] - poses[ref][1],
                                poses[cur][2]};
    scan_extract_points(&s_ref_scan, ref_local[0], ref_local[1], ref_local[2], ref_cloud);
    scan_extract_points(&s_cur_scan, cur_local[0], cur_local[1], cur_local[2], cur_cloud);

    constraint cons;
    bool ok = comp_constr(ref_cloud, cur_cloud, ref_local, cur_local,
                          (int16_t)ref, (int16_t)cur, &cons);
    if (ok) {
        s_constraints[s_num_constraints] = cons;
        s_kinds[s_num_constraints] = kind;
        s_num_constraints++;
    }
    NS_YIELD(); /* runs in flight too */
    return ok;
}

size_t slam_optimize(float poses[GRAPH_MAX_POSES][3], slam_report_t *report) {
    slam_report_t rep;
    memset(&rep, 0, sizeof(rep));
    uint64_t t_start_us = NS_NOW_US();

    /* Snapshot of the pose count: poses recorded while this runs (in flight)
     * are left alone and re-anchored by the caller on commit. */
    size_t total_poses = slam_graph_pose_count();
    if (total_poses > GRAPH_MAX_POSES) total_poses = GRAPH_MAX_POSES;
    rep.poses = total_poses;
    if (total_poses < 2) {
        NS_LOGW(TAG, "Not enough poses in graph (%d) to run SLAM", (int)total_poses);
        if (report) *report = rep;
        return 0;
    }

    for (size_t i = 0; i < total_poses; i++) {
        slam_pose_t p = {0};
        slam_graph_get_pose(i, &p);
        poses[i][0] = p.x;
        poses[i][1] = p.y;
        poses[i][2] = p.yaw;
    }

    s_num_constraints = 0;
    memset(s_line_rejects, 0, sizeof(s_line_rejects));

    /* 1. Sequential ICP between consecutive poses (refines the odometry
     *    backbone). */
    for (size_t i = 1; i < total_poses && s_num_constraints < SLAM_MAX_CONSTRAINTS; i++) {
        rep.sequential_attempts++;
        if (try_constraint(poses, i - 1, i, SLAM_CONSTRAINT_SEQUENTIAL)) {
            rep.sequential_accepted++;
        }
    }

    /* 2. Loop closures for non-adjacent poses. One constraint per pose,
     *    against its closest earlier pose: accepting every nearby pair let
     *    dozens of 20x-weighted constraints overpower odometry. */
    for (size_t i = LOOP_MIN_NODE_GAP; i < total_poses && s_num_constraints < SLAM_MAX_CONSTRAINTS; i++) {
        int best_j = -1;
        float best_dist = LOOP_MAX_DIST_M;
        for (size_t j = 0; j + LOOP_MIN_NODE_GAP <= i; j++) {
            float dist = hypotf(poses[i][0] - poses[j][0], poses[i][1] - poses[j][1]);
            float dyaw = fabsf(wrap_pi(poses[i][2] - poses[j][2]));
            if (dist < best_dist && dyaw < LOOP_MAX_DYAW_RAD) {
                best_dist = dist;
                best_j = (int)j;
            }
        }
        if (best_j < 0) continue;
        rep.loop_attempts++;
        if (try_constraint(poses, (size_t)best_j, i, SLAM_CONSTRAINT_LOOP)) {
            rep.loop_accepted++;
        }
    }

    uint64_t t_solve_us = NS_NOW_US();
    rep.icp_ms = (uint32_t)((t_solve_us - t_start_us) / 1000u);

    NS_LOGI(TAG, "Running LS-SLAM on %d poses with %d constraints (seq %u/%u, loop %u/%u)",
            (int)total_poses, (int)s_num_constraints,
            rep.sequential_accepted, rep.sequential_attempts,
            rep.loop_accepted, rep.loop_attempts);

    /* Odometry edges from the raw odometry poses, not from the current
     * estimates: in flight the estimates already contain earlier corrections,
     * which ETH's formulation would count again on every pass. */
    for (size_t i = 1; i < total_poses; i++) {
        slam_pose_t a = {0}, b = {0};
        slam_graph_get_odom(i - 1, &a);
        slam_graph_get_odom(i, &b);
        float from[3] = {a.x, a.y, a.yaw}, to[3] = {b.x, b.y, b.yaw};
        measurement_from_poses(from, to, s_odometry[i - 1].z);
    }
    ls_slam_set_weights(&s_weights);
    rep.solved = ls_slam_odometry((float *)poses, s_odometry, NULL, s_constraints,
                                  (int16_t)total_poses, (int16_t)s_num_constraints);
    for (size_t i = 0; i < total_poses; i++) {
        poses[i][2] = wrap_pi(poses[i][2]);
    }
    rep.solve_ms = (uint32_t)((NS_NOW_US() - t_solve_us) / 1000u);

    if (!rep.solved) {
        NS_LOGW(TAG, "LS-SLAM solve rejected (sparse capacity or non-finite); poses unchanged");
    }
    NS_LOGI(TAG, "LS-SLAM done: icp %u ms, solve %u ms",
            (unsigned)rep.icp_ms, (unsigned)rep.solve_ms);

    if (report) *report = rep;
    return total_poses;
}

void slam_commit_poses(const float poses[GRAPH_MAX_POSES][3], size_t n) {
    for (size_t i = 0; i < n; i++) {
        slam_pose_t p = {poses[i][0], poses[i][1], poses[i][2]};
        slam_graph_set_pose(i, &p);
    }
}

void slam_set_icp_method(slam_icp_method_t method) { s_icp_method = method; }

void slam_set_weights(const ls_slam_weights_t *weights) { s_weights = *weights; }
ls_slam_weights_t slam_get_weights(void) { return s_weights; }
slam_icp_method_t slam_get_icp_method(void) { return s_icp_method; }

void slam_icp_line_rejects(uint16_t out[SLAM_ICP_REJECT_REASONS]) {
    memcpy(out, s_line_rejects, sizeof(s_line_rejects));
}

size_t slam_static_bytes(void) {
    return sizeof(s_ref_scan) + sizeof(s_cur_scan) + sizeof(s_ref_buf) + sizeof(s_cur_buf) +
           sizeof(s_constraints) + sizeof(s_kinds) + sizeof(s_odometry) + ICP_STATIC_BYTES +
           ls_slam_static_bytes();
}

size_t slam_last_constraints(const constraint **constraints, const uint8_t **kinds) {
    if (constraints) *constraints = s_constraints;
    if (kinds) *kinds = s_kinds;
    return s_num_constraints;
}
