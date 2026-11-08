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

#ifndef NODE_H
#define NODE_H

#include <stdint.h>
#include <stdlib.h>

typedef struct {
    int16_t index;
    int16_t neighbors;
    int8_t visited;
} node;

#endif // NODE_H
