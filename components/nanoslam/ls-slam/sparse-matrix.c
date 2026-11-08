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
#include <string.h>
#include <math.h>

#include "sparse-matrix.h"
#include "nanoslam_port.h"

void insert_element(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j, float element) {
    if (fabsf(element) < 0.00001f)
        return;

    // If the new element is higher order than previously added elements
    if (spMat->idx1 == 0) {
        spMat->highestOrder = -1;
        spMat->idx0 = -1;
    }

    if (i * N + j <= spMat->highestOrder) {
        int32_t rowStart = spMat->rowCnt[i];
        int32_t rowEnd = spMat->rowCnt[i + 1];
        int32_t k_last = rowStart;
        for (int32_t k = rowStart; k < rowEnd; k++) {
            k_last = rowEnd;
            if (spMat->col[k] == j) { // Already present
                spMat->data[k] = element;
                spMat->col[k] = j;
                return;
            }
            if (spMat->col[k] > j) {
                k_last = k;
                break;
            }
        }

        if (spMat->idx0 + 1 >= MAX_SIZE_SP_MAT) {
            spMat->overflow = 1;
            return;
        }

        memmove(&(spMat->data[k_last + 1]), &(spMat->data[k_last]), (spMat->idx0 - k_last + 1) * sizeof(float));
        memmove(&(spMat->col[k_last + 1]), &(spMat->col[k_last]), (spMat->idx0 - k_last + 1) * sizeof(int16_t));

        spMat->data[k_last] = element;
        spMat->col[k_last] = j;
        for (int16_t l = i + 1; l <= spMat->idx1; l++)
            spMat->rowCnt[l]++;
        spMat->idx0++;
        return;
    }

    if (spMat->idx0 + 1 >= MAX_SIZE_SP_MAT) {
        spMat->overflow = 1;
        return;
    }

    spMat->idx0++;
    spMat->data[spMat->idx0] = element;
    spMat->col[spMat->idx0] = j;
    for (int16_t k = spMat->idx1; k <= i; k++)
        spMat->rowCnt[k + 1] = spMat->rowCnt[k];

    spMat->rowCnt[i + 1]++;
    spMat->idx1 = i + 1;

    spMat->highestOrder = i * N + j;
}

void add_to_element(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j, float value) {
    int32_t rowStart = spMat->rowCnt[i];
    int32_t rowEnd = spMat->rowCnt[i + 1];

    for (int32_t k = rowStart; k < rowEnd; k++) {
        if (spMat->col[k] == j) {
            spMat->data[k] += value;
            return;
        }
    }

    insert_element(spMat, N, i, j, value);
}

int16_t already_added(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j) {
    int32_t rowStart = spMat->rowCnt[i];
    int32_t rowEnd = spMat->rowCnt[i + 1];

    for (int32_t k = rowStart; k < rowEnd; k++) {
        if (spMat->col[k] > j)
            return 0;
        if (spMat->col[k] == j)
            return 1;
    }
    return 0;
}

float get_element(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j) {
    int32_t rowStart = spMat->rowCnt[i];
    int32_t rowEnd = spMat->rowCnt[i + 1];

    for (int32_t k = rowStart; k < rowEnd; k++) {
        if (spMat->col[k] == j)
            return spMat->data[k];
        if (spMat->col[k] > j)
            break;
    }

    return 0.0f;
}

float get_element_sym(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j) {
    if (i < j) {
        int16_t aux = i;
        i = j;
        j = aux;
    }

    int32_t rowStart = spMat->rowCnt[i];
    int32_t rowEnd = spMat->rowCnt[i + 1];

    for (int32_t k = rowStart; k < rowEnd; k++) {
        if (spMat->col[k] == j)
            return spMat->data[k];
        if (spMat->col[k] > j)
            break;
    }

    return 0.0f;
}

void sparse_from_dense(int16_t N, float mat[N][N], sparseMatrix *spMat) {
    for (int16_t i = 0; i < N; i++)
        for (int16_t j = 0; j < N; j++)
            if (fabsf(mat[i][j]) > 0.00001f)
                insert_element(spMat, N, i, j, mat[i][j]);
}

void transpose_lower_triangular(sparseMatrix *spMat, int16_t N) {
    static sparseMatrix *s_tr; /* PSRAM when available; one solve at a time */
    if (s_tr == NULL) s_tr = (sparseMatrix *)ns_large_calloc(1, sizeof(sparseMatrix));
    if (s_tr == NULL) {
        spMat->overflow = 1;
        return;
    }
    memset(s_tr, 0, sizeof(sparseMatrix));

    for (int16_t i = 0; i < N; i++)
        for (int32_t k = spMat->rowCnt[i]; k < spMat->rowCnt[i + 1]; k++)
            insert_element(s_tr, N, spMat->col[k], i, spMat->data[k]);

    if (s_tr->overflow) spMat->overflow = 1;
    memcpy(spMat, s_tr, sizeof(sparseMatrix));
}
