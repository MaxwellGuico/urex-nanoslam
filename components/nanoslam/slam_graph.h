/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See components/nanoslam/README.md.
 *
 * Pose and scan storage, adapted from swarm_graph in ETH-PBL/NanoSLAM via
 * Duncan Lau's ESP32-S3 port. Single drone: nodes are indexed 0..n-1 and the
 * multi-drone ID/broadcast layer is not ported.
 */

#ifndef SLAM_GRAPH_H
#define SLAM_GRAPH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config_size.h"
#include "scan.h"

#define GRAPH_MAX_POSES MAX_POSES

typedef struct {
    float x, y, yaw; /* NED: x = north, y = east, yaw clockwise from north */
} slam_pose_t;

/* Allocates scan storage (PSRAM when available). Safe to call repeatedly. */
bool slam_graph_init(void);

/* Forget all poses and scans. */
void slam_graph_reset(void);

/* Appends a node with its scan. pose is the current (map-frame) estimate,
 * odom the raw odometry pose it was recorded at; odometry edges are always
 * taken from the raw poses. Returns the node index, or -1 when full. The
 * scan's pose field is overwritten with the new index. */
int slam_graph_add(const slam_pose_t *pose, const slam_pose_t *odom, const scan_t *scan);

size_t slam_graph_pose_count(void);

bool slam_graph_get_pose(size_t index, slam_pose_t *pose);
bool slam_graph_set_pose(size_t index, const slam_pose_t *pose);
bool slam_graph_get_odom(size_t index, slam_pose_t *odom);

/* Copies node index's scan into scan. */
bool slam_graph_get_scan(size_t index, scan_t *scan);

/* Bytes of pose + scan storage (the scans are heap-allocated, PSRAM if present). */
size_t slam_graph_storage_bytes(void);

#endif /* SLAM_GRAPH_H */
