#pragma once

/* Bridge between the flight tasks and the NanoSLAM component
 * (components/nanoslam), adapted from Duncan Lau's slam_bridge.c.
 *
 * Frames
 *   odom : PX4 local NED with estimator resets removed (drone_state_t.slam_x/y)
 *   map  : SLAM frame. Equal to odom when recording starts; every committed
 *          optimisation updates the correction D so that map = D (+) odom.
 *
 * The task records one pose + 400 ms ToF scan every
 * CONFIG_NANOSLAM_RECORD_INTERVAL_MS while armed, optimises in the background
 * every CONFIG_NANOSLAM_INFLIGHT_EVERY_N poses, and after disarming runs a
 * final optimisation and uploads the graph to the laptop over UDP
 * (slam_upload.h). It never sends setpoints and does not feed navigation. */

#include <stdbool.h>
#include <stddef.h>

#define SLAM_TASK_CORE      0
#define SLAM_TASK_PRIORITY  1
#define SLAM_TASK_STACK     8192

/* Call once before spawning tasks. */
void slam_bridge_init(void);

/* FreeRTOS task entry. */
void slam_task(void *arg);

/* odom <-> map using the latest committed correction. Thread-safe. Not used
 * by navigation yet. */
void slam_bridge_odom_to_map(float odom_x, float odom_y, float *map_x, float *map_y);
void slam_bridge_map_to_odom(float map_x, float map_y, float *odom_x, float *odom_y);

size_t slam_bridge_get_pose_count(void);
bool slam_bridge_is_optimizing(void);
