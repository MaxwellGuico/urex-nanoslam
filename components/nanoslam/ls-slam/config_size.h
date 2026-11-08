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

#ifndef CONFIG_SIZE_H
#define CONFIG_SIZE_H

/* 64 poses x 3 states. MAX_SIZE_SP_MAT bounds the non-zeros of H and of its
 * Cholesky factor. ETH uses 9 * MAX_SIZE, but on 2026-11-08 a 34-pose flight
 * in a small room (26 loop closures) needed 1748 > 1728 and the final solve
 * was rejected. A full lower triangle can never overflow; the three matrices
 * (~111 KB each) are heap-allocated in PSRAM. */
#define MAX_POSES           64
#define MAX_SIZE            (3 * MAX_POSES)
#define MAX_SIZE_SP_MAT     (MAX_SIZE * (MAX_SIZE + 1) / 2)

#endif // CONFIG_SIZE_H
