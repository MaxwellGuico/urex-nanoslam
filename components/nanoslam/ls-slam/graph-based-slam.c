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
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "utils_math.h"
#include "config_size.h"
#include "sparse-matrix.h"
#include "rcm-sparse-matrix.h"
#include "graph-based-slam.h"
#include "nanoslam_port.h"

/* H and its Cholesky factor, allocated once (PSRAM when available) */
static sparseMatrix *s_H;
static sparseMatrix *s_L;

void v2t(float pose[3], float T[3][3]) {
    float c = cosf(pose[2]);
    float s = sinf(pose[2]);
    T[0][0] = c;  T[0][1] = -s; T[0][2] = pose[0];
    T[1][0] = s;  T[1][1] = c;  T[1][2] = pose[1];
    T[2][0] = 0;  T[2][1] = 0;  T[2][2] = 1;
}

void t2v(float T[3][3], float v[3]) {
    float x = T[0][2];
    float y = T[1][2];
    float theta = atan2f(T[1][0], T[0][0]);
    v[0] = x; v[1] = y; v[2] = theta;
}

void linearize_pose_pose_constraint(float *x1, float *x2, float *z, float A[3][3], float B[3][3], float *e) {
    // Compute e
    float Zij[3][3];
    float Xi[3][3];
    float Xj[3][3];

    v2t(z, Zij);
    v2t(x1, Xi);
    v2t(x2, Xj);

    matinv_3x3(Zij, Zij);
    matinv_3x3(Xi, Xi);
    matmul_3x3(Zij, Xi, Xi);
    matmul_3x3(Xi, Xj, Xi);

    t2v(Xi, e);

    // compute A & B
    float s = sinf(x1[2]);
    float c = cosf(x1[2]);

    float s2 = sinf(z[2]);
    float c2 = cosf(z[2]);

    float RiT[2][2] = {{c, s}, {-s, c}};
    float RijT[2][2] = {{c2, s2}, {-s2, c2}};
    float dRiT_dTheta[2][2] = {{-s, c}, {-c, -s}};
    float delta_t[2] = {x2[0] - x1[0], x2[1] - x1[1]};

    float aux0[2][2];
    float aux1[2];

    matmul_2x2(RijT, dRiT_dTheta, aux0);
    matmul_2x2x1(aux0, delta_t, aux1);
    matmul_2x2(RijT, RiT, aux0);

    A[0][0] = -aux0[0][0]; A[0][1] = -aux0[0][1]; A[0][2] = aux1[0];
    A[1][0] = -aux0[1][0]; A[1][1] = -aux0[1][1]; A[1][2] = aux1[1];
    A[2][0] = 0;           A[2][1] = 0;           A[2][2] = -1;

    B[0][0] = aux0[0][0]; B[0][1] = aux0[0][1]; B[0][2] = 0;
    B[1][0] = aux0[1][0]; B[1][1] = aux0[1][1]; B[1][2] = 0;
    B[2][0] = 0;          B[2][1] = 0;          B[2][2] = 1;
}

/* Per-axis standard deviations of odometry and ICP constraint edges. The ETH
 * original uses unit odometry information and 20x isotropic ICP information
 * (sigma 1 and 1/sqrt(20) in every axis, metres and radians alike), which
 * suits a Crazyflie with poor yaw. A PX4 gyro heading drifts ~0.03 deg/s and
 * is far more precise than a single ICP match, so callers set per-axis values
 * (ls_slam_set_weights). Defaults reproduce ETH. */
static ls_slam_weights_t s_weights = {
    .odom_sigma_xy = 1.0f, .odom_sigma_yaw = 1.0f,
    .icp_sigma_xy = 0.2236068f, .icp_sigma_yaw = 0.2236068f,
};
/* Information on pose 0 (gauge). Must dominate every edge: ETH used 1000 with
 * unit odometry information; with per-axis weights edges reach ~1e4. */
static float s_prior = 1000.0f;

void ls_slam_set_weights(const ls_slam_weights_t *w) {
    s_weights = *w;
    float max_info = 0.0f;
    const float sig[4] = {w->odom_sigma_xy, w->odom_sigma_yaw, w->icp_sigma_xy, w->icp_sigma_yaw};
    for (int k = 0; k < 4; k++) {
        const float info = 1.0f / (sig[k] * sig[k]);
        if (info > max_info) max_info = info;
    }
    s_prior = 1000.0f * (max_info > 1.0f ? max_info : 1.0f);
}

/* Scale residual rows by 1/sigma so that A^T A, A^T B, B^T B and A^T e are
 * the information-weighted blocks. x and y share one sigma, which keeps the
 * block structure build_H_and_b relies on (rotation^T * w * rotation = w I). */
static void weigh_rows(float A[3][3], float B[3][3], float e[3], float sigma_xy, float sigma_yaw) {
    const float w[3] = {1.0f / sigma_xy, 1.0f / sigma_xy, 1.0f / sigma_yaw};
    for (int k = 0; k < 3; k++) {
        for (int c = 0; c < 3; c++) {
            A[k][c] *= w[k];
            B[k][c] *= w[k];
        }
        e[k] *= w[k];
    }
}

void build_H_and_b(float *x,
                   sparseMatrix *H,
                   float *b,
                   measurement *measurements,
                   const uint8_t *skip,
                   constraint *constraints,
                   int16_t nr_of_poses,
                   int16_t nr_of_constr) {
    float z[3];
    float A[3][3], B[3][3], e[3];

    int16_t dim = 3 * nr_of_poses;

    insert_element(H, dim, 0, 0, s_prior);
    insert_element(H, dim, 1, 1, s_prior);
    insert_element(H, dim, 2, 0, 0);
    insert_element(H, dim, 2, 1, 0);
    insert_element(H, dim, 2, 2, s_prior);

    for (int16_t i = 1; i < nr_of_poses; i++) {
        if (skip && skip[i - 1]) continue;

        float xi[3] = {x[3 * i - 3], x[3 * i - 2], x[3 * i - 1]};
        float xj[3] = {x[3 * i], x[3 * i + 1], x[3 * i + 2]};
        memcpy(z, measurements[i - 1].z, sizeof(z));

        linearize_pose_pose_constraint(xi, xj, z, A, B, e);
        weigh_rows(A, B, e, s_weights.odom_sigma_xy, s_weights.odom_sigma_yaw);
        float bi[3], bj[3];
        float Hii[3][3], Hjj[3][3], Hij[3][3];
        transpose_and_mult_3x3x1(A, e, bi);
        transpose_and_mult_3x3x1(B, e, bj);
        transpose_and_mult_3x3(A, A, Hii);
        transpose_and_mult_3x3(A, B, Hij);
        transpose_and_mult_3x3(B, B, Hjj);

        add_to_element(H, dim, 3 * (i - 1) + 0, 3 * (i - 1) + 0, Hii[0][0]);
        add_to_element(H, dim, 3 * (i - 1) + 1, 3 * (i - 1) + 1, Hii[1][1]);
        add_to_element(H, dim, 3 * (i - 1) + 2, 3 * (i - 1) + 0, Hii[2][0]);
        add_to_element(H, dim, 3 * (i - 1) + 2, 3 * (i - 1) + 1, Hii[2][1]);
        add_to_element(H, dim, 3 * (i - 1) + 2, 3 * (i - 1) + 2, Hii[2][2]);

        insert_element(H, dim, 3 * i + 0, 3 * (i - 1) + 0, Hij[0][0]);
        insert_element(H, dim, 3 * i + 0, 3 * i + 0, Hjj[0][0]);
        insert_element(H, dim, 3 * i + 1, 3 * (i - 1) + 1, Hij[1][1]);
        insert_element(H, dim, 3 * i + 1, 3 * i + 1, Hjj[1][1]);
        insert_element(H, dim, 3 * i + 0, 3 * (i - 1) + 2, Hij[2][0]);
        insert_element(H, dim, 3 * i + 1, 3 * (i - 1) + 2, Hij[2][1]);
        insert_element(H, dim, 3 * i + 2, 3 * (i - 1) + 2, Hij[2][2]);
        insert_element(H, dim, 3 * i + 2, 3 * i + 0, 0);
        insert_element(H, dim, 3 * i + 2, 3 * i + 1, 0);
        insert_element(H, dim, 3 * i + 2, 3 * i + 2, Hjj[2][2]);

        for (int16_t i0 = 0; i0 < 3; i0++) {
            b[3 * (i - 1) + i0] += -bi[i0];
            b[3 * i + i0] += -bj[i0];
        }
    }

    for (int16_t i = 0; i < nr_of_constr; i++) {
        int fromIdx = constraints[i].fromIdx;
        int toIdx = constraints[i].toIdx;
        const float info = 1.0f; /* weights applied in weigh_rows */
        memcpy(z, constraints[i].z, sizeof(z));

        float xi[3] = {x[3 * fromIdx], x[3 * fromIdx + 1], x[3 * fromIdx + 2]};
        float xj[3] = {x[3 * toIdx], x[3 * toIdx + 1], x[3 * toIdx + 2]};

        linearize_pose_pose_constraint(xi, xj, z, A, B, e);
        weigh_rows(A, B, e, s_weights.icp_sigma_xy, s_weights.icp_sigma_yaw);
        float bi[3], bj[3];
        float Hii[3][3], Hjj[3][3], Hij[3][3];
        transpose_and_mult_3x3x1(A, e, bi);
        transpose_and_mult_3x3x1(B, e, bj);
        transpose_and_mult_3x3(A, A, Hii);
        transpose_and_mult_3x3(A, B, Hij);
        transpose_and_mult_3x3(B, B, Hjj);

        for (int16_t i0 = 0; i0 < 3; i0++) {
            add_to_element(H, dim, 3 * fromIdx + i0, 3 * fromIdx + i0, info * Hii[i0][i0]);
            add_to_element(H, dim, 3 * toIdx + i0, 3 * toIdx + i0, info * Hjj[i0][i0]);
            /* add, not insert: a sequential-ICP constraint shares its pose
             * pair with an odometry edge, whose block is already present */
            add_to_element(H, dim, 3 * fromIdx + i0, 3 * toIdx + i0, info * Hij[i0][i0]);
        }
        add_to_element(H, dim, 3 * fromIdx + 2, 3 * fromIdx + 0, info * Hii[2][0]);
        add_to_element(H, dim, 3 * fromIdx + 2, 3 * fromIdx + 1, info * Hii[2][1]);

        add_to_element(H, dim, 3 * fromIdx + 2, 3 * toIdx + 0, info * Hij[2][0]);
        add_to_element(H, dim, 3 * fromIdx + 2, 3 * toIdx + 1, info * Hij[2][1]);

        for (int16_t i0 = 0; i0 < 3; i0++) {
            b[3 * fromIdx + i0] += -bi[i0] * info;
            b[3 * toIdx + i0] += -bj[i0] * info;
        }
    }
}

void forward_substitution(sparseMatrix *L, float *b, int N) {
    float sol[N];
    memset(sol, 0, sizeof(sol));
    float diag0 = get_element(L, N, 0, 0);
    sol[0] = (diag0 != 0.0f) ? (b[0] / diag0) : 0.0f;
    for (int i = 1; i < N; i++) {
        float sum = 0;
        for (int32_t k = L->rowCnt[i]; k < L->rowCnt[i + 1]; k++)
            if (i > L->col[k])
                sum += L->data[k] * sol[L->col[k]];
        float diagi = get_element(L, N, i, i);
        sol[i] = (diagi != 0.0f) ? ((1.0f / diagi) * (b[i] - sum)) : 0.0f;
    }

    memcpy(b, sol, sizeof(sol));
}

void backward_substitution(sparseMatrix *L, float *b, int N) {
    float sol[N];
    memset(sol, 0, sizeof(sol));
    float diagN = get_element(L, N, N - 1, N - 1);
    sol[N - 1] = (diagN != 0.0f) ? (b[N - 1] / diagN) : 0.0f;
    for (int i = N - 2; i >= 0; i--) {
        float sum = 0;
        for (int32_t k = L->rowCnt[i]; k < L->rowCnt[i + 1]; k++)
            if (i < L->col[k])
                sum += L->data[k] * sol[L->col[k]];
        float diagi = get_element(L, N, i, i);
        sol[i] = (diagi != 0.0f) ? ((1.0f / diagi) * (b[i] - sum)) : 0.0f;
    }

    memcpy(b, sol, sizeof(sol));
}

static float sparse_dot_product(sparseMatrix *S, int16_t row_a, int16_t row_b, int16_t max_col) {
    float sum = 0;
    int16_t i = row_a;
    int16_t j = row_b;

    int16_t start = S->rowCnt[i];
    for (int32_t k = S->rowCnt[j]; k < S->rowCnt[j + 1]; k++) {
        int16_t colIdxJ = S->col[k];
        if (colIdxJ > max_col)
            break;
        for (int32_t o = start; o < S->rowCnt[i + 1]; o++) {
            int16_t colIdxI = S->col[o];
            if (colIdxJ == colIdxI) {
                sum += S->data[k] * S->data[o];
                start = o;
                break;
            }
            if (colIdxI > colIdxJ) {
                start = o;
                break;
            }
        }
    }
    return sum;
}

static void cholesky_sparse_2(sparseMatrix *H, int16_t N, int16_t *perm) {
    int16_t i, j;
    int32_t k;
    if (s_L == NULL) s_L = (sparseMatrix *)ns_large_calloc(1, sizeof(sparseMatrix));
    if (s_L == NULL) {
        H->overflow = 1;
        return;
    }
    sparseMatrix *L = s_L; /* not re-entrant: one solve at a time */
    memset(L, 0, sizeof(sparseMatrix));

    for (j = 0; j < N; j++) {
        float sum = 0;
        for (k = 0; k < j; k++) {
            float el = get_element(L, N, j, k);
            sum += el * el;
        }

        float diff = get_element_sym(H, N, perm[j], perm[j]) - sum;
        float element = (diff > 0.0f) ? sqrtf(diff) : 0.001f;
        insert_element(L, N, j, j, element);

        for (i = j + 1; i < N; i++) {
            sum = sparse_dot_product(L, j, i, j - 1);

            float Hij = get_element_sym(H, N, perm[i], perm[j]) - sum;
            if (Hij == 0.0f)
                element = 0.0f;
            else
                element = (1.0f / get_element(L, N, j, j)) * Hij;

            insert_element(L, N, i, j, element);
        }
    }
    memcpy(H, L, sizeof(sparseMatrix));
}

float test_linear_system(sparseMatrix *H, float *x, float *b, int N) {
    float res[N];
    memset(res, 0, sizeof(res));
    for (int16_t i = 0; i < N; i++) {
        for (int32_t k = H->rowCnt[i]; k < H->rowCnt[i + 1]; k++) {
            res[i] += H->data[k] * x[H->col[k]];
            if (i > H->col[k])
                res[H->col[k]] += H->data[k] * x[i];
        }
        res[i] -= b[i];
    }

    float error = 0.0f;
    for (int16_t i = 0; i < N; i++)
        error += (float) fabsf(res[i]);

    return error;
}

bool solve_for_x(sparseMatrix *H, float *b, int16_t *perm, int16_t size) {
    // Cholesky Decomposition
    cholesky_sparse_2(H, size, perm);
    if (H->overflow)
        return false;

    permute_array(b, perm, size);

    /* L * L.T * x = b;   we note L.T * x = y
       Solves L * y = b; */
    forward_substitution(H, b, size);
    float *y = b;

    // Solves L.T * x = y
    transpose_lower_triangular(H, size);
    backward_substitution(H, y, size);
    float *x = y;

    // Inverse permute the solution
    permutation_inverse(perm, size);
    permute_array(x, perm, size);
    return !H->overflow;
}

void measurement_from_poses(float *x_from, float *x_to, float *z) {
    for (int16_t k = 0; k < 3; k++)
        z[k] = x_to[k] - x_from[k];

    float s = sinf(-x_from[2]);
    float c = cosf(-x_from[2]);
    float R[2][2] = {{c, -s}, {s, c}};
    matmul_2x2x1(R, z, z);
}

void measurements_from_poses(float *x, measurement *measurements, int nr_of_poses) {
    float z[3];
    for (int16_t i = 1; i < nr_of_poses; i++) {
        float xi[3] = {x[3 * i - 3], x[3 * i - 2], x[3 * i - 1]};
        float xj[3] = {x[3 * i], x[3 * i + 1], x[3 * i + 2]};
        measurement_from_poses(xi, xj, z);

        measurements[i - 1].z[0] = z[0];
        measurements[i - 1].z[1] = z[1];
        measurements[i - 1].z[2] = z[2];
    }
}

/* Per-pose arrays in ls_slam (static). The three sparse matrices - H, the
 * Cholesky factor and the transpose scratch - are heap: ls_slam_heap_bytes. */
size_t ls_slam_static_bytes(void) {
    return MAX_SIZE * (sizeof(int16_t) + 2 * sizeof(float)) + MAX_POSES * sizeof(measurement);
}

size_t ls_slam_heap_bytes(void) {
    return 3 * sizeof(sparseMatrix);
}

bool ls_slam(float *x, uint8_t *skip, constraint *constraints, int16_t nr_of_poses, int16_t nr_of_constr) {
    return ls_slam_odometry(x, NULL, skip, constraints, nr_of_poses, nr_of_constr);
}

bool ls_slam_odometry(float *x, const measurement *odometry, uint8_t *skip,
                      constraint *constraints, int16_t nr_of_poses, int16_t nr_of_constr) {
    /* Static like ETH's sp_mat0/array0: keeps ~10 KB per matrix off the
     * caller's stack. Not re-entrant: only one optimisation at a time. */
    static int16_t R[MAX_SIZE];
    static measurement measurements[MAX_POSES];
    static float b[MAX_SIZE];
    static float x_start[MAX_SIZE];

    if (nr_of_poses < 2 || nr_of_poses > MAX_POSES)
        return false;
    if (s_H == NULL) s_H = (sparseMatrix *)ns_large_calloc(1, sizeof(sparseMatrix));
    if (s_H == NULL)
        return false;
    sparseMatrix *H = s_H;
    const int16_t dim = (int16_t)(3 * nr_of_poses);
    memcpy(x_start, x, (size_t)dim * sizeof(float));

    if (odometry != NULL) {
        memcpy(measurements, odometry, (size_t)(nr_of_poses - 1) * sizeof(measurement));
    } else {
        measurements_from_poses(x, measurements, nr_of_poses);
    }

    for (int16_t k = 0; k < 3; k++) {
        memset(H, 0, sizeof(sparseMatrix));
        memset(b, 0, (size_t)dim * sizeof(float));

        build_H_and_b(x, H, b, measurements, skip, constraints, nr_of_poses, nr_of_constr);
        if (H->overflow) {
            memcpy(x, x_start, (size_t)dim * sizeof(float));
            return false;
        }
        rcm(H, R, dim);
        if (!solve_for_x(H, b, R, dim)) {
            memcpy(x, x_start, (size_t)dim * sizeof(float));
            return false;
        }

        for (int16_t i = 0; i < dim; i++) {
            if (!isfinite(b[i])) {
                memcpy(x, x_start, (size_t)dim * sizeof(float));
                return false;
            }
            x[i] += b[i];
        }
    }
    return true;
}
