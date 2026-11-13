#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { float x, y; } experiment_xy_t;
/* Home + N length + E width + S length + W width back to home. */
experiment_xy_t experiment_corner(unsigned corner, float length, float width);
/* Waypoints of a mission, x forward / y right of the nose at takeoff. Waypoint 0 is
 * takeoff; the last one is where the vehicle lands. mode is
 * EXPERIMENT_TAKEOFF_LAND (1), EXPERIMENT_RECTANGLE (2, takeoff at a corner)
 * or EXPERIMENT_RECTANGLE_CENTRED (3, takeoff mid bottom edge). */
unsigned experiment_waypoint_count(uint8_t mode);
experiment_xy_t experiment_waypoint(uint8_t mode, unsigned index, float length, float width);
/* Rotate a takeoff-frame offset (x forward, y right of the nose at takeoff)
 * into PX4 local NED (x north, y east). heading is PX4's yaw at takeoff,
 * clockwise from north. Without a compass PX4's north is arbitrary, so the
 * mission is laid out relative to the nose: place the drone square to the
 * room and the rectangle is square to the room. */
experiment_xy_t experiment_rotate(experiment_xy_t body, float heading);
bool experiment_settled(float distance, float speed, float tolerance);
experiment_xy_t experiment_step(experiment_xy_t current, experiment_xy_t goal,
                                float speed, float dt);
