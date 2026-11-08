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

#include <stdio.h>
#include <string.h>

#include "circular-queue.h"

int16_t init_queue(circularQueue *queue) {
    memset(queue, 0, sizeof(circularQueue));
    return 1;
}

int16_t is_full(circularQueue *queue) {
    return (queue->size >= MAX_SIZE) ? 1 : 0;
}

int16_t is_empty(circularQueue *queue) {
    return (queue->size == 0) ? 1 : 0;
}

int16_t push_to_queue(circularQueue *queue, node element) {
    if (is_full(queue))
        return 0;

    queue->nodes[queue->tail] = element;
    queue->tail = (queue->tail + 1) % MAX_SIZE;
    queue->size++;

    return 1;
}

int16_t pop_from_queue(circularQueue *queue, node *element) {
    if (is_empty(queue))
        return 0;

    *element = queue->nodes[queue->head];
    queue->head = (queue->head + 1) % MAX_SIZE;
    queue->size--;

    return 1;
}
