/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See components/nanoslam/README.md.
 *
 * Adapted from swarm_graph in ETH-PBL/NanoSLAM via Duncan Lau's ESP32-S3
 * port. Modified for the UREX nus_flight_app.
 */

#include "slam_graph.h"

#include <string.h>

#include "nanoslam_port.h"

static const char *TAG = "slam_graph";

static ns_mutex_t s_mutex;
static bool s_ready;
static size_t s_num_poses;
static slam_pose_t s_poses[GRAPH_MAX_POSES];
static slam_pose_t s_odom[GRAPH_MAX_POSES];
/* 64 scans x ~700 B; allocated in PSRAM when available */
static scan_t *s_scans;

bool slam_graph_init(void) {
    if (s_ready) return true;
    s_mutex = ns_mutex_create();
    s_scans = (scan_t *)ns_large_calloc(GRAPH_MAX_POSES, sizeof(scan_t));
    if (!s_mutex || !s_scans) {
        NS_LOGW(TAG, "Could not allocate pose graph storage");
        return false;
    }
    s_num_poses = 0;
    s_ready = true;
    NS_LOGI(TAG, "Pose graph ready (max %d poses, %u B of scans)",
            GRAPH_MAX_POSES, (unsigned)(GRAPH_MAX_POSES * sizeof(scan_t)));
    return true;
}

void slam_graph_reset(void) {
    if (!s_ready) return;
    ns_mutex_lock(s_mutex);
    s_num_poses = 0;
    memset(s_poses, 0, sizeof(s_poses));
    memset(s_odom, 0, sizeof(s_odom));
    ns_mutex_unlock(s_mutex);
}

int slam_graph_add(const slam_pose_t *pose, const slam_pose_t *odom, const scan_t *scan) {
    if (!s_ready || pose == NULL || odom == NULL || scan == NULL) return -1;
    ns_mutex_lock(s_mutex);
    int index = -1;
    if (s_num_poses < GRAPH_MAX_POSES) {
        index = (int)s_num_poses;
        s_poses[index] = *pose;
        s_odom[index] = *odom;
        s_scans[index] = *scan;
        s_scans[index].pose = (uint16_t)index;
        s_num_poses++;
    }
    ns_mutex_unlock(s_mutex);
    return index;
}

size_t slam_graph_pose_count(void) {
    if (!s_ready) return 0;
    ns_mutex_lock(s_mutex);
    size_t n = s_num_poses;
    ns_mutex_unlock(s_mutex);
    return n;
}

bool slam_graph_get_pose(size_t index, slam_pose_t *pose) {
    if (!s_ready || pose == NULL) return false;
    ns_mutex_lock(s_mutex);
    bool ok = index < s_num_poses;
    if (ok) *pose = s_poses[index];
    ns_mutex_unlock(s_mutex);
    return ok;
}

bool slam_graph_set_pose(size_t index, const slam_pose_t *pose) {
    if (!s_ready || pose == NULL) return false;
    ns_mutex_lock(s_mutex);
    bool ok = index < s_num_poses;
    if (ok) s_poses[index] = *pose;
    ns_mutex_unlock(s_mutex);
    return ok;
}

size_t slam_graph_storage_bytes(void) {
    return sizeof(s_poses) + sizeof(s_odom) + GRAPH_MAX_POSES * sizeof(scan_t);
}

bool slam_graph_get_odom(size_t index, slam_pose_t *odom) {
    if (!s_ready || odom == NULL) return false;
    ns_mutex_lock(s_mutex);
    bool ok = index < s_num_poses;
    if (ok) *odom = s_odom[index];
    ns_mutex_unlock(s_mutex);
    return ok;
}

bool slam_graph_get_scan(size_t index, scan_t *scan) {
    if (!s_ready || scan == NULL) return false;
    ns_mutex_lock(s_mutex);
    bool ok = index < s_num_poses;
    if (ok) *scan = s_scans[index];
    ns_mutex_unlock(s_mutex);
    return ok;
}
