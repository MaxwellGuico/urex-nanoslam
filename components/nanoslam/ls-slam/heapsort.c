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
#include <stdlib.h>

#include "node.h"
#include "heapsort.h"

static void swap_nodes(node *a, node *b) {
    node temp = *a;
    *a = *b;
    *b = temp;
}

static void heapify(node *nodes, int16_t N, int16_t i) {
    int16_t largest = i;
    int16_t left = 2 * i + 1;
    int16_t right = 2 * i + 2;

    if (left < N && nodes[left].neighbors > nodes[largest].neighbors)
        largest = left;

    if (right < N && nodes[right].neighbors > nodes[largest].neighbors)
        largest = right;

    if (largest != i) {
        swap_nodes(&nodes[i], &nodes[largest]);
        heapify(nodes, N, largest);
    }
}

void heapsort_nodes(node *nodes, int16_t N) {
    for (int16_t i = N / 2 - 1; i >= 0; i--)
        heapify(nodes, N, i);

    for (int16_t i = N - 1; i >= 0; i--) {
        swap_nodes(&nodes[0], &nodes[i]);
        heapify(nodes, i, 0);
    }
}
