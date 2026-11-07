#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { float x, y; } experiment_xy_t;
/* Home + N length + E width + S length + W width back to home. */
experiment_xy_t experiment_corner(unsigned corner, float length, float width);
bool experiment_settled(float distance, float speed, float tolerance);
experiment_xy_t experiment_step(experiment_xy_t current, experiment_xy_t goal,
                                float speed, float dt);
