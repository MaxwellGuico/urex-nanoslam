/* Host unit tests for the NanoSLAM component. */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "graph-based-slam.h"
#include "icp.h"
#include "icp_line.h"
#include "scan.h"
#include "slam.h"
#include "slam_graph.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG(x) ((float)(x) * (float)M_PI / 180.0f)

static float wrap(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* pose_j = pose_i (+) z, NED, yaw clockwise */
static void compose(const float a[3], const float z[3], float out[3]) {
    float c = cosf(a[2]), s = sinf(a[2]);
    out[0] = a[0] + c * z[0] - s * z[1];
    out[1] = a[1] + s * z[0] + c * z[1];
    out[2] = wrap(a[2] + z[2]);
}

/* --- ICP ------------------------------------------------------------------ */

static void test_icp_recovers_transform_on_scattered_points(void) {
    const int n = 60;
    icp_points_t *q = points_alloc(n), *p = points_alloc(n);
    srand(7);
    for (int i = 0; i < n; i++) {
        q->items[i].x = 2.0f * rand() / (float)RAND_MAX;
        q->items[i].y = 2.0f * rand() / (float)RAND_MAX;
    }
    const float yaw = DEG(5), tx = 0.10f, ty = 0.05f;
    const float c = cosf(yaw), s = sinf(yaw);
    for (int i = 0; i < n; i++) {
        float x = q->items[i].x, y = q->items[i].y;
        p->items[i].x = c * x - s * y + tx;
        p->items[i].y = s * x + c * y + ty;
    }
    icp_point_t t;
    float rot;
    icp(q, p, 30, 0.6f, &t, &rot);
    /* icp(Q, P) returns the transform that moves P onto Q: the inverse */
    assert(fabsf(rot + yaw) < DEG(0.05));
    assert(fabsf(t.x + (c * tx + s * ty)) < 0.002f);
    assert(fabsf(t.y + (-s * tx + c * ty)) < 0.002f);
    free(q);
    free(p);
}

/* --- ls_slam -------------------------------------------------------------- */

/* Translation-only chain: odometry says 1.0 m per step, ICP constraints say
 * 1.1 m. The problem is linear, so the weighted optimum per edge is
 * (1 * 1.0 + 20 * 1.1) / 21. This fails if a constraint overwrites, rather
 * than adds to, the odometry edge's off-diagonal block. */
static void test_sequential_constraints_combine_with_odometry(void) {
    const ls_slam_weights_t eth = SLAM_WEIGHTS_ETH;
    ls_slam_set_weights(&eth);
    enum { N = 6 };
    float x[3 * N];
    constraint cons[N - 1];
    for (int i = 0; i < N; i++) {
        x[3 * i] = (float)i;
        x[3 * i + 1] = 0.0f;
        x[3 * i + 2] = 0.0f;
    }
    for (int i = 1; i < N; i++) {
        cons[i - 1].fromIdx = (int16_t)i;     /* newer -> older */
        cons[i - 1].toIdx = (int16_t)(i - 1);
        cons[i - 1].z[0] = -1.1f;             /* older pose seen from newer */
        cons[i - 1].z[1] = 0.0f;
        cons[i - 1].z[2] = 0.0f;
    }
    assert(ls_slam(x, NULL, cons, N, N - 1));
    const float step = (1.0f + 20.0f * 1.1f) / 21.0f;
    for (int i = 0; i < N; i++) {
        assert(fabsf(x[3 * i] - step * (float)i) < 0.002f);
        assert(fabsf(x[3 * i + 1]) < 0.002f);
        assert(fabsf(x[3 * i + 2]) < 0.001f);
    }
}

/* Square loop with heading drift; one loop constraint closes it. */
static void test_loop_constraint_closes_square(void) {
    const ls_slam_weights_t eth = SLAM_WEIGHTS_ETH;
    ls_slam_set_weights(&eth);
    enum { SIDE = 8, N = 4 * SIDE + 1 };
    float truth[N][3], x[3 * N];
    const float step = 0.25f;
    truth[0][0] = truth[0][1] = truth[0][2] = 0.0f;
    for (int i = 1; i < N; i++) {
        float z[3] = {step, 0.0f, (i % SIDE == 0) ? DEG(90) : 0.0f};
        compose(truth[i - 1], z, truth[i]);
    }
    /* Odometry: same steps but 1 deg of extra yaw per step */
    memcpy(x, truth[0], sizeof(float) * 3);
    for (int i = 1; i < N; i++) {
        float z[3] = {step, 0.0f, (i % SIDE == 0) ? DEG(90) : 0.0f};
        z[2] += DEG(1);
        compose(&x[3 * (i - 1)], z, &x[3 * i]);
    }
    float before = hypotf(x[3 * (N - 1)] - truth[N - 1][0], x[3 * (N - 1) + 1] - truth[N - 1][1]);

    /* Loop: last pose sees pose 0 at its true relative pose */
    constraint c = {.fromIdx = N - 1, .toIdx = 0};
    measurement_from_poses(truth[N - 1], truth[0], c.z);
    assert(ls_slam(x, NULL, &c, N, 1));
    float after = hypotf(x[3 * (N - 1)] - truth[N - 1][0], x[3 * (N - 1) + 1] - truth[N - 1][1]);
    printf("square loop: end error %.3f m -> %.3f m\n", before, after);
    assert(before > 0.5f);
    assert(after < 0.15f);
}

/* Long-range loop closures across the whole graph cause heavy Cholesky
 * fill-in; the matrices hold a full lower triangle, so this must solve. */
static void test_dense_loop_closures_always_fit(void) {
    const ls_slam_weights_t eth = SLAM_WEIGHTS_ETH;
    ls_slam_set_weights(&eth);
    enum { N = MAX_POSES };
    static float x[3 * N];
    static constraint cons[SLAM_MAX_CONSTRAINTS];
    for (int i = 0; i < N; i++) {
        x[3 * i] = 0.3f * (float)i;
        x[3 * i + 1] = 0.1f * sinf((float)i);
        x[3 * i + 2] = 0.01f * (float)i;
    }
    srand(3);
    for (int k = 0; k < SLAM_MAX_CONSTRAINTS; k++) {
        int a = rand() % N, b = rand() % N;
        if (a == b) b = (a + N / 2) % N;
        cons[k].fromIdx = (int16_t)(a > b ? a : b);
        cons[k].toIdx = (int16_t)(a > b ? b : a);
        cons[k].z[0] = 0.05f;
        cons[k].z[1] = -0.02f;
        cons[k].z[2] = 0.01f;
    }
    assert(ls_slam(x, NULL, cons, N, SLAM_MAX_CONSTRAINTS));
    for (int i = 0; i < 3 * N; i++) assert(isfinite(x[i]));
}

/* Exact final solve of the 2026-11-08 rectangle flight (34 poses, 33
 * sequential + 26 loop constraints). With ETH's 9 * MAX_SIZE capacity the
 * Cholesky factor needed 1748 > 1728 entries and the drone rejected it. */
static void test_flight_20261108_final_solve(void) {
    const ls_slam_weights_t eth = SLAM_WEIGHTS_ETH;
    ls_slam_set_weights(&eth);
    static float x[3 * MAX_POSES];
    static constraint cons[SLAM_MAX_CONSTRAINTS];
    FILE *f = fopen(NANOSLAM_TEST_DATA "/flight-20261108-poses.txt", "r");
    assert(f);
    int n = 0;
    while (n < MAX_POSES && fscanf(f, "%f %f %f", &x[3 * n], &x[3 * n + 1], &x[3 * n + 2]) == 3) n++;
    fclose(f);
    f = fopen(NANOSLAM_TEST_DATA "/flight-20261108-constraints.txt", "r");
    assert(f);
    int m = 0, from, to, kind;
    float z0, z1, z2;
    while (m < SLAM_MAX_CONSTRAINTS &&
           fscanf(f, "%d %d %d %f %f %f", &from, &to, &kind, &z0, &z1, &z2) == 6) {
        cons[m] = (constraint){.fromIdx = (int16_t)from, .toIdx = (int16_t)to, .z = {z0, z1, z2}};
        m++;
    }
    fclose(f);
    assert(n == 34 && m == 59);
    assert(ls_slam(x, NULL, cons, (int16_t)n, (int16_t)m));
    for (int i = 0; i < 3 * n; i++) assert(isfinite(x[i]));
    printf("flight 2026-11-08 final solve: ok (%d poses, %d constraints)\n", n, m);
}

/* Weights at ETH's values reproduce ETH's information exactly: unit odometry,
 * 20x ICP. Same chain as above but through the weighted path. */
static void test_weights_reproduce_eth(void) {
    enum { N = 6 };
    float x[3 * N];
    constraint cons[N - 1];
    measurement odo[N - 1];
    for (int i = 0; i < N; i++) { x[3 * i] = (float)i; x[3 * i + 1] = 0; x[3 * i + 2] = 0; }
    for (int i = 1; i < N; i++) {
        cons[i - 1] = (constraint){.fromIdx = (int16_t)i, .toIdx = (int16_t)(i - 1), .z = {-1.1f, 0, 0}};
        odo[i - 1] = (measurement){.z = {1.0f, 0, 0}};
    }
    const ls_slam_weights_t eth = SLAM_WEIGHTS_ETH;
    ls_slam_set_weights(&eth);
    assert(ls_slam_odometry(x, odo, NULL, cons, N, N - 1));
    const float step = (1.0f + 20.0f * 1.1f) / 21.0f;
    for (int i = 0; i < N; i++) assert(fabsf(x[3 * i] - step * (float)i) < 0.002f);
    /* per-axis: odometry 8 cm, ICP 3 cm -> weights 156 and 1111 */
    const ls_slam_weights_t per_axis = SLAM_WEIGHTS_DEFAULT;
    ls_slam_set_weights(&per_axis);
    for (int i = 0; i < N; i++) { x[3 * i] = (float)i; x[3 * i + 1] = 0; x[3 * i + 2] = 0; }
    assert(ls_slam_odometry(x, odo, NULL, cons, N, N - 1));
    const float wo = 1 / (0.08f * 0.08f), wi = 1 / (0.03f * 0.03f);
    const float step2 = (wo * 1.0f + wi * 1.1f) / (wo + wi);
    for (int i = 0; i < N; i++) assert(fabsf(x[3 * i] - step2 * (float)i) < 0.002f);
}

/* PX4-like odometry: exact heading, scale error; ICP: exact translation but
 * +-2 deg heading noise. Per-axis weights must keep the heading close to
 * odometry's; ETH's 20x isotropic weighting follows the noisy ICP heading. */
static void test_per_axis_weights_keep_good_heading(void) {
    enum { N = 30 };
    static float truth[N][3], x_eth[3 * N], x_axis[3 * N];
    static constraint cons[N - 1];
    static measurement odo[N - 1];
    srand(11);
    truth[0][0] = truth[0][1] = truth[0][2] = 0;
    for (int i = 1; i < N; i++) {
        float z[3] = {0.3f, 0.0f, 0.0f};
        compose(truth[i - 1], z, truth[i]);
    }
    for (int i = 1; i < N; i++) {
        measurement_from_poses(truth[i - 1], truth[i], odo[i - 1].z);
        odo[i - 1].z[0] *= 1.05f;                              /* odometry: 5% scale */
        float zt[3];
        measurement_from_poses(truth[i], truth[i - 1], zt);    /* ICP: newer -> older */
        zt[2] += DEG(2) * (2.0f * rand() / (float)RAND_MAX - 1.0f);  /* +-2 deg */
        cons[i - 1] = (constraint){.fromIdx = (int16_t)i, .toIdx = (int16_t)(i - 1), .z = {zt[0], zt[1], zt[2]}};
    }
    for (int i = 0; i < N; i++) {          /* start from odometry */
        float p[3] = {0.3f * 1.05f * i, 0, 0};
        memcpy(&x_eth[3 * i], p, sizeof(p));
        memcpy(&x_axis[3 * i], p, sizeof(p));
    }
    const ls_slam_weights_t eth = SLAM_WEIGHTS_ETH, axis = SLAM_WEIGHTS_DEFAULT;
    ls_slam_set_weights(&eth);
    assert(ls_slam_odometry(x_eth, odo, NULL, cons, N, N - 1));
    ls_slam_set_weights(&axis);
    assert(ls_slam_odometry(x_axis, odo, NULL, cons, N, N - 1));
    float yaw_eth = 0, yaw_axis = 0;
    for (int i = 0; i < N; i++) {
        yaw_eth = fmaxf(yaw_eth, fabsf(wrap(x_eth[3 * i + 2])));
        yaw_axis = fmaxf(yaw_axis, fabsf(wrap(x_axis[3 * i + 2])));
    }
    float end_eth = hypotf(x_eth[3 * (N - 1)] - truth[N - 1][0], x_eth[3 * (N - 1) + 1] - truth[N - 1][1]);
    float end_axis = hypotf(x_axis[3 * (N - 1)] - truth[N - 1][0], x_axis[3 * (N - 1) + 1] - truth[N - 1][1]);
    printf("heading test: max yaw error ETH %.2f deg, per-axis %.2f deg; end error ETH %.3f m, per-axis %.3f m\n",
           yaw_eth * 180 / M_PI, yaw_axis * 180 / M_PI, end_eth, end_axis);
    assert(yaw_axis < DEG(1.0) && yaw_axis < 0.25f * yaw_eth);
    assert(end_axis < 0.10f);   /* ICP still removes most of the 5% scale error (0.44 m) */
}

/* --- scan ----------------------------------------------------------------- */

static void test_reduce_frame_median_and_spread(void) {
    int16_t dist[SCAN_ZONES];
    uint8_t status[SCAN_ZONES];
    for (int i = 0; i < SCAN_ZONES; i++) {
        dist[i] = 1000;
        status[i] = 5;
    }
    /* column 0: rows 2-5 = 1000, 1010, 1020, 1030 -> median 1015 */
    dist[2 * 8] = 1000; dist[3 * 8] = 1010; dist[4 * 8] = 1020; dist[5 * 8] = 1030;
    /* column 1: rows disagree by > 100 mm -> rejected */
    dist[2 * 8 + 1] = 800; dist[3 * 8 + 1] = 1200;
    /* column 2: only one valid row -> rejected */
    status[3 * 8 + 2] = 255; status[4 * 8 + 2] = 255; status[5 * 8 + 2] = 255;
    /* column 3: outer rows only would be ignored; too close (chassis) */
    for (int r = 2; r < 6; r++) dist[r * 8 + 3] = 150;

    int16_t out[SCAN_NUM_COLS];
    int rejected = scan_reduce_frame(dist, status, 8, out);
    assert(out[0] == 1015);
    assert(out[1] == -1);
    assert(out[2] == -1);
    assert(out[3] == -1);
    assert(out[4] == 1000);
    assert(rejected == 2);
}

static void test_extract_points_geometry(void) {
    float yaw[SCAN_NUM_DIRS];
    for (int k = 0; k < SCAN_NUM_DIRS; k++) yaw[k] = DEG(90 * (k == 1)); /* sensor 1 faces right */
    scan_set_geometry(yaw, 0.0f);
    static scan_t sc;
    memset(&sc, 0xff, sizeof(sc)); /* all -1 */
    for (int f = 0; f < SCAN_NUM_FRAMES; f++) {
        sc.frames[f].dx = sc.frames[f].dy = sc.frames[f].dyaw = 0.0f;
    }
    sc.frames[0].dists[1][3] = 1000; /* sensor 1, just clockwise of centre */
    static uint8_t buf[POINTS_SIZEOF(POINTS_IN_SCAN)];
    icp_points_t *pts = (icp_points_t *)buf;
    /* Drone at (1, 2) facing north: sensor 1 looks east */
    scan_extract_points(&sc, 1.0f, 2.0f, 0.0f, pts);
    assert(pts->num == 1);
    /* column 3 centre is +2.8 deg clockwise of the sensor axis */
    float a = DEG(90 + 2.8125);
    assert(fabsf(pts->items[0].x - (1.0f + cosf(a))) < 1e-3f);
    assert(fabsf(pts->items[0].y - (2.0f + sinf(a))) < 1e-3f);
}

/* Room perimeter 3 x 2.5 m sampled every 5 cm, starting `phase` metres along */
static int room_perimeter(icp_points_t *out, float phase) {
    const float W = 3.0f, H = 2.5f, step = 0.05f, total = 2 * (W + H);
    int n = 0;
    for (float d = phase; d < total && n < ICP_MAX_POINTS; d += step) {
        float x, y;
        if (d < W) { x = d; y = 0; }
        else if (d < W + H) { x = W; y = d - W; }
        else if (d < 2 * W + H) { x = 2 * W + H - d; y = H; }
        else { x = 0; y = total - d; }
        out->items[n].x = x;
        out->items[n].y = y;
        n++;
    }
    out->num = (size_t)n;
    return n;
}

/* The real ToF case: the new scan hits the same walls at different spots.
 * Point-to-line must recover the motion; point-to-point is biased. */
static void test_icp_line_recovers_motion_on_walls(void) {
    static uint8_t qb[POINTS_SIZEOF(ICP_MAX_POINTS)], pb[POINTS_SIZEOF(ICP_MAX_POINTS)];
    icp_points_t *q = (icp_points_t *)qb, *p = (icp_points_t *)pb;
    room_perimeter(q, 0.0f);
    room_perimeter(p, 0.025f); /* half a sample along every wall */
    const float yaw = DEG(4), tx = 0.08f, ty = -0.05f, c = cosf(yaw), s = sinf(yaw);
    for (size_t i = 0; i < p->num; i++) {
        float x = p->items[i].x, y = p->items[i].y;
        p->items[i].x = c * x - s * y + tx;
        p->items[i].y = s * x + c * y + ty;
    }
    /* expected: the inverse transform */
    const float ex = -(c * tx + s * ty), ey = -(-s * tx + c * ty);

    icp_point_t t;
    float rot;
    icp_line_info_t info;
    assert(icp_point_to_line(q, p, NULL, &t, &rot, &info));
    float line_rot_err = fabsf(rot + yaw), line_t_err = hypotf(t.x - ex, t.y - ey);

    icp(q, p, 30, 0.6f, &t, &rot);
    float pp_rot_err = fabsf(rot + yaw), pp_t_err = hypotf(t.x - ex, t.y - ey);
    printf("walls, resampled: point-to-line err %.3f deg / %.1f mm (%d it, rmse %.1f mm); "
           "point-to-point err %.3f deg / %.1f mm\n",
           line_rot_err * 180 / M_PI, line_t_err * 1000, info.iterations, info.rmse_m * 1000,
           pp_rot_err * 180 / M_PI, pp_t_err * 1000);
    assert(line_rot_err < DEG(0.05));
    assert(line_t_err < 0.002f);
}

/* One straight wall: sliding along it is unobservable, so reject. */
static void test_icp_line_rejects_single_wall(void) {
    static uint8_t qb[POINTS_SIZEOF(ICP_MAX_POINTS)], pb[POINTS_SIZEOF(ICP_MAX_POINTS)];
    icp_points_t *q = (icp_points_t *)qb, *p = (icp_points_t *)pb;
    q->num = p->num = 60;
    for (int i = 0; i < 60; i++) {
        q->items[i].x = 0.05f * i; q->items[i].y = 1.0f;
        p->items[i].x = 0.05f * i + 0.2f; p->items[i].y = 1.01f;
    }
    icp_point_t t;
    float rot;
    icp_line_info_t info;
    assert(!icp_point_to_line(q, p, NULL, &t, &rot, &info));
    assert(info.status == ICP_LINE_DEGENERATE);
}

/* 4x4 frame: 16 zones row-major, centre rows 1-2 must both be valid and agree */
static void test_reduce_frame_4x4(void) {
    int16_t dist[SCAN_ZONES];
    uint8_t status[SCAN_ZONES];
    for (int i = 0; i < SCAN_ZONES; i++) { dist[i] = -1; status[i] = 255; }
    for (int i = 0; i < 16; i++) { dist[i] = 1200; status[i] = 5; }
    dist[1 * 4 + 0] = 1000; dist[2 * 4 + 0] = 1020;   /* col 0 -> 1010 */
    dist[1 * 4 + 1] = 900;  dist[2 * 4 + 1] = 1300;   /* col 1 disagrees -> rejected */
    status[2 * 4 + 2] = 255;                          /* col 2 one row -> rejected */
    int16_t out[SCAN_NUM_COLS];
    int rejected = scan_reduce_frame(dist, status, 4, out);
    assert(out[0] == 1010 && out[1] == -1 && out[2] == -1 && out[3] == 1200);
    for (int c = 4; c < SCAN_NUM_COLS; c++) assert(out[c] == -1);
    assert(rejected == 2);
}

/* 4x4 columns are 11.25 deg wide: column 1 centre is 5.625 deg clockwise */
static void test_extract_points_4x4(void) {
    float yaw[SCAN_NUM_DIRS] = {0};
    scan_set_geometry(yaw, 0.0f);
    scan_set_columns(4);
    static scan_t sc;
    memset(&sc, 0xff, sizeof(sc));
    for (int f = 0; f < SCAN_NUM_FRAMES; f++) sc.frames[f].dx = sc.frames[f].dy = sc.frames[f].dyaw = 0.0f;
    sc.frames[0].dists[0][1] = 2000;
    sc.frames[0].dists[0][5] = 2000; /* beyond 4 columns: ignored */
    static uint8_t buf[POINTS_SIZEOF(POINTS_IN_SCAN)];
    icp_points_t *pts = (icp_points_t *)buf;
    scan_extract_points(&sc, 0.0f, 0.0f, 0.0f, pts);
    assert(pts->num == 1);
    float a = DEG(5.625);
    assert(fabsf(pts->items[0].x - 2.0f * cosf(a)) < 1e-3f);
    assert(fabsf(pts->items[0].y - 2.0f * sinf(a)) < 1e-3f);
    scan_set_columns(8);
}

/* --- end to end ----------------------------------------------------------- */

#define ROOM_N 3.0f
#define ROOM_E 2.5f

/* Range from (x, y) along NED bearing b to the walls of a rectangular room */
static float room_range(float x, float y, float b) {
    float dx = cosf(b), dy = sinf(b), t = INFINITY;
    if (dx > 1e-6f) t = fminf(t, (ROOM_N - x) / dx);
    if (dx < -1e-6f) t = fminf(t, -x / dx);
    if (dy > 1e-6f) t = fminf(t, (ROOM_E - y) / dy);
    if (dy < -1e-6f) t = fminf(t, -y / dy);
    return t;
}

static void synth_scan(const float pose[3], const float sensor_yaw[SCAN_NUM_DIRS], scan_t *sc) {
    const int cols = scan_get_columns();
    const float step = -DEG(45) / (float)cols;
    for (int f = 0; f < SCAN_NUM_FRAMES; f++) {
        sc->frames[f].dx = sc->frames[f].dy = sc->frames[f].dyaw = 0.0f;
        for (int d = 0; d < SCAN_NUM_DIRS; d++) {
            float angle = -(float)(cols / 2) * step + step / 2.0f;
            for (int col = 0; col < SCAN_NUM_COLS; col++, angle += step) {
                if (col >= cols) { sc->frames[f].dists[d][col] = -1; continue; }
                float r = room_range(pose[0], pose[1], pose[2] + sensor_yaw[d] + angle);
                int mm = (int)lrintf(r * 1000.0f);
                sc->frames[f].dists[d][col] =
                    (mm >= SCAN_MIN_VALID_MM && mm <= SCAN_MAX_VALID_MM) ? (int16_t)mm : -1;
            }
        }
    }
}

/* Fly a loop inside a room with drifting odometry, then optimise. */
static void test_end_to_end_room_loop(slam_icp_method_t method, int columns) {
    slam_set_icp_method(method);
    scan_set_columns(columns);
    float sensor_yaw[SCAN_NUM_DIRS];
    for (int k = 0; k < SCAN_NUM_DIRS; k++) sensor_yaw[k] = -DEG(45 * k);
    scan_set_geometry(sensor_yaw, 0.0f);
    assert(slam_graph_init());
    slam_graph_reset();

    /* Rectangle 0.8..2.2 N x 0.8..1.7 E, ~0.35 m between poses, yaw fixed */
    static const float way[][2] = {{0.8f, 0.8f}, {2.2f, 0.8f}, {2.2f, 1.7f}, {0.8f, 1.7f}, {0.8f, 0.8f}};
    static float truth[GRAPH_MAX_POSES][3], odom[GRAPH_MAX_POSES][3];
    int n = 0;
    for (int leg = 0; leg < 4; leg++) {
        float len = hypotf(way[leg + 1][0] - way[leg][0], way[leg + 1][1] - way[leg][1]);
        int steps = (int)ceilf(len / 0.35f);
        for (int s = 0; s < steps; s++) {
            float u = (float)s / (float)steps;
            truth[n][0] = way[leg][0] + u * (way[leg + 1][0] - way[leg][0]);
            truth[n][1] = way[leg][1] + u * (way[leg + 1][1] - way[leg][1]);
            truth[n][2] = DEG(10);
            n++;
        }
    }
    truth[n][0] = way[4][0]; truth[n][1] = way[4][1]; truth[n][2] = DEG(10); n++;

    /* Odometry: 0.4 deg yaw drift and 4% scale error per step */
    memcpy(odom[0], truth[0], sizeof(odom[0]));
    for (int i = 1; i < n; i++) {
        float z[3];
        measurement_from_poses(truth[i - 1], truth[i], z);
        z[0] *= 1.04f; z[1] *= 1.04f; z[2] += DEG(0.4);
        compose(odom[i - 1], z, odom[i]);
    }

    static scan_t sc;
    for (int i = 0; i < n; i++) {
        synth_scan(truth[i], sensor_yaw, &sc);
        slam_pose_t p = {odom[i][0], odom[i][1], odom[i][2]};
        assert(slam_graph_add(&p, &p, &sc) == i);
    }

    float err_before = 0, err_after = 0;
    for (int i = 0; i < n; i++) err_before += hypotf(odom[i][0] - truth[i][0], odom[i][1] - truth[i][1]);

    static float poses[GRAPH_MAX_POSES][3];
    slam_report_t rep;
    assert(slam_optimize(poses, &rep) == (size_t)n);
    assert(rep.solved);
    for (int i = 0; i < n; i++) err_after += hypotf(poses[i][0] - truth[i][0], poses[i][1] - truth[i][1]);
    err_before /= (float)n;
    err_after /= (float)n;
    float end_before = hypotf(odom[n - 1][0] - truth[n - 1][0], odom[n - 1][1] - truth[n - 1][1]);
    float end_after = hypotf(poses[n - 1][0] - truth[n - 1][0], poses[n - 1][1] - truth[n - 1][1]);
    printf("room loop (%s, %dx%d): %d poses, seq %u/%u, loop %u/%u, mean error %.3f -> %.3f m, "
           "end error %.3f -> %.3f m\n",
           method == SLAM_ICP_POINT_TO_LINE ? "point-to-line" : "point-to-point",
           columns, columns, n, rep.sequential_accepted, rep.sequential_attempts, rep.loop_accepted,
           rep.loop_attempts, err_before, err_after, end_before, end_after);
    assert(rep.sequential_accepted > 0);
    assert(err_after < err_before);

    slam_commit_poses(poses, (size_t)n);
    slam_pose_t back;
    assert(slam_graph_get_pose((size_t)n - 1, &back));
    assert(back.x == poses[n - 1][0]);
}

/* In flight the drone optimises, commits and optimises again. With odometry
 * taken from the raw poses a second pass must not move the result further. */
static void test_repeated_passes_do_not_compound(void) {
    test_end_to_end_room_loop(SLAM_ICP_POINT_TO_LINE, 8);   /* leaves a graph */
    static float first[GRAPH_MAX_POSES][3], second[GRAPH_MAX_POSES][3];
    slam_report_t rep;
    size_t n = slam_optimize(first, &rep);
    slam_commit_poses(first, n);
    assert(slam_optimize(second, &rep) == n);
    float moved = 0;
    for (size_t i = 0; i < n; i++) moved = fmaxf(moved, hypotf(second[i][0] - first[i][0], second[i][1] - first[i][1]));
    printf("repeated pass moved poses by at most %.4f m\n", moved);
    assert(moved < 0.01f);
}

int main(void) {
    test_icp_recovers_transform_on_scattered_points();
    test_sequential_constraints_combine_with_odometry();
    test_loop_constraint_closes_square();
    test_dense_loop_closures_always_fit();
    test_flight_20261108_final_solve();
    test_reduce_frame_median_and_spread();
    test_extract_points_geometry();
    test_icp_line_recovers_motion_on_walls();
    test_icp_line_rejects_single_wall();
    test_reduce_frame_4x4();
    test_extract_points_4x4();
    test_end_to_end_room_loop(SLAM_ICP_POINT_TO_POINT, 8);
    test_end_to_end_room_loop(SLAM_ICP_POINT_TO_LINE, 8);
    test_end_to_end_room_loop(SLAM_ICP_POINT_TO_LINE, 4);
    test_weights_reproduce_eth();
    test_per_axis_weights_keep_good_heading();
    test_repeated_passes_do_not_compound();
    printf("nanoslam tests passed; scan_t=%u B, sparseMatrix=%u B\n",
           (unsigned)sizeof(scan_t), (unsigned)sizeof(sparseMatrix));
    return 0;
}
