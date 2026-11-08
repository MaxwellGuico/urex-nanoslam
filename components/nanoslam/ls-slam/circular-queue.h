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

#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include <stdint.h>
#include "node.h"
#include "config_size.h"

typedef struct {
    node nodes[MAX_SIZE];
    int16_t head;
    int16_t tail;
    int16_t size;
} circularQueue;

int16_t init_queue(circularQueue *queue);
int16_t is_full(circularQueue *queue);
int16_t is_empty(circularQueue *queue);
int16_t push_to_queue(circularQueue *queue, node element);
int16_t pop_from_queue(circularQueue *queue, node *element);

#endif // CIRCULAR_QUEUE_H
