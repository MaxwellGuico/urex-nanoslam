/*
 * Point-to-line ICP (see icp_line.h). Mirrors icp() in laptop/offline_slam.py:
 * 3 cm voxel filter, degenerate-geometry check, local line normals from up to
 * 5 neighbours within 0.45 m, nearest-neighbour matches gated by distance and
 * a valid normal, 90th-percentile trimming, one source point per target point,
 * Gauss-Newton on (tx, ty, theta), observability and convergence checks, then
 * overlap and RMSE gates. All buffers are static and O(N^2) searches are used
 * (N <= 320), which is cheaper than building a tree at this size.
 */

#include "icp_line.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define VOXEL_M            0.03f
#define NORMAL_K           5
#define NORMAL_RADIUS_M    0.45f
#define NORMAL_MAX_RATIO   0.1f    /* eigenvalue ratio: points must lie on a line */
#define DEGENERATE_RATIO   0.01f   /* whole cloud this elongated = a single wall */
#define TRIM_QUANTILE      0.9f
#define CONVERGED_STEP     1e-4f
/* After the last iteration, a match still changing by less than this is
 * accepted: on the 2026-11-08 flights most "not converged" matches had 0.8-0.98
 * overlap and were alternating between near-identical correspondence sets with
 * 0.1-4.5 mm steps, well below ToF range noise (~1-2 cm). */
#define SETTLED_STEP_M     0.005f
#define SETTLED_STEP_RAD   0.005f
#define UNOBSERVABLE_RATIO 1e-6f   /* (sigma_min / sigma_max)^2 < (1e-3)^2 */

static icp_point_t s_src[ICP_MAX_POINTS], s_dst[ICP_MAX_POINTS], s_moved[ICP_MAX_POINTS];
static icp_point_t s_normal[ICP_MAX_POINTS];
static uint8_t s_normal_ok[ICP_MAX_POINTS];
static int16_t s_match[ICP_MAX_POINTS];
static float s_dist[ICP_MAX_POINTS];
static float s_err[ICP_MAX_POINTS];
static float s_sorted[ICP_MAX_POINTS];
static int16_t s_best_src[ICP_MAX_POINTS];

icp_line_config_t icp_line_default_config(void) {
    icp_line_config_t c = {
        .max_distance_m = 0.25f,
        .min_overlap = 0.5f,
        .max_rmse_m = 0.08f,
        .min_points = 12,
        .max_iterations = 40,
    };
    return c;
}

const char *icp_line_status_name(icp_line_status_t status) {
    switch (status) {
    case ICP_LINE_ACCEPTED: return "accepted";
    case ICP_LINE_TOO_FEW_POINTS: return "too_few_points";
    case ICP_LINE_DEGENERATE: return "degenerate_geometry";
    case ICP_LINE_TOO_FEW_MATCHES: return "too_few_correspondences";
    case ICP_LINE_UNOBSERVABLE: return "unobservable_geometry";
    case ICP_LINE_NOT_CONVERGED: return "not_converged";
    case ICP_LINE_LOW_OVERLAP: return "low_overlap";
    case ICP_LINE_HIGH_RMSE: return "high_rmse";
    }
    return "unknown";
}

/* Keeps the first point in each 3 cm cell, in input order */
static int voxel_filter(const icp_points_t *in, icp_point_t *out) {
    static int32_t keys[ICP_MAX_POINTS][2];
    int n = 0;
    for (size_t i = 0; i < in->num; i++) {
        int32_t kx = (int32_t)floorf(in->items[i].x / VOXEL_M);
        int32_t ky = (int32_t)floorf(in->items[i].y / VOXEL_M);
        bool dup = false;
        for (int j = 0; j < n; j++) {
            if (keys[j][0] == kx && keys[j][1] == ky) { dup = true; break; }
        }
        if (dup) continue;
        keys[n][0] = kx;
        keys[n][1] = ky;
        out[n++] = in->items[i];
    }
    return n;
}

/* Eigen-decomposition of a symmetric 2x2 [[a, b], [b, c]] */
static void eig2(float a, float b, float c, float *lmin, float *lmax, icp_point_t *vmin) {
    float mean = 0.5f * (a + c);
    float r = sqrtf(0.25f * (a - c) * (a - c) + b * b);
    *lmin = mean - r;
    *lmax = mean + r;
    if (fabsf(b) > 1e-12f) {
        float vx = *lmin - c, vy = b;
        float n = hypotf(vx, vy);
        if (n < 1e-12f) { vx = b; vy = *lmin - a; n = hypotf(vx, vy); }
        vmin->x = vx / n;
        vmin->y = vy / n;
    } else if (a <= c) {
        vmin->x = 1.0f; vmin->y = 0.0f;
    } else {
        vmin->x = 0.0f; vmin->y = 1.0f;
    }
}

static void covariance(const icp_point_t *p, const int *idx, int n, float *a, float *b, float *c) {
    float mx = 0, my = 0;
    for (int i = 0; i < n; i++) { mx += p[idx ? idx[i] : i].x; my += p[idx ? idx[i] : i].y; }
    mx /= (float)n; my /= (float)n;
    float sxx = 0, sxy = 0, syy = 0;
    for (int i = 0; i < n; i++) {
        float dx = p[idx ? idx[i] : i].x - mx, dy = p[idx ? idx[i] : i].y - my;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    float d = n > 1 ? (float)(n - 1) : 1.0f;
    *a = sxx / d; *b = sxy / d; *c = syy / d;
}

static bool degenerate(const icp_point_t *p, int n) {
    float a, b, c, lmin, lmax;
    icp_point_t v;
    covariance(p, NULL, n, &a, &b, &c);
    eig2(a, b, c, &lmin, &lmax, &v);
    return lmax <= 1e-10f || lmin / lmax < DEGENERATE_RATIO;
}

/* Line normal at every target point from its <= 5 nearest neighbours (itself
 * included) within 0.45 m; isolated returns and corners get no normal. */
static void compute_normals(const icp_point_t *q, int n) {
    for (int i = 0; i < n; i++) {
        int nn[NORMAL_K];
        float nd[NORMAL_K];
        int count = 0;
        for (int j = 0; j < n; j++) {
            float dx = q[j].x - q[i].x, dy = q[j].y - q[i].y;
            float d = dx * dx + dy * dy;
            if (d > NORMAL_RADIUS_M * NORMAL_RADIUS_M) continue;
            if (count < NORMAL_K) {
                nn[count] = j; nd[count] = d; count++;
            } else {
                int worst = 0;
                for (int k = 1; k < NORMAL_K; k++) if (nd[k] > nd[worst]) worst = k;
                if (d < nd[worst]) { nn[worst] = j; nd[worst] = d; }
            }
        }
        s_normal_ok[i] = 0;
        if (count < 3) continue;
        float a, b, c, lmin, lmax;
        covariance(q, nn, count, &a, &b, &c);
        eig2(a, b, c, &lmin, &lmax, &s_normal[i]);
        if (lmax > 1e-8f && lmin / lmax < NORMAL_MAX_RATIO) s_normal_ok[i] = 1;
    }
}

static void nearest(const icp_point_t *from, int nf, const icp_point_t *to, int nt,
                    int16_t *idx, float *dist) {
    for (int i = 0; i < nf; i++) {
        float best = INFINITY;
        int16_t bi = -1;
        for (int j = 0; j < nt; j++) {
            float dx = to[j].x - from[i].x, dy = to[j].y - from[i].y;
            float d = dx * dx + dy * dy;
            if (d < best) { best = d; bi = (int16_t)j; }
        }
        idx[i] = bi;
        dist[i] = sqrtf(best);
    }
}

static void apply(const icp_point_t *in, int n, float th, float tx, float ty, icp_point_t *out) {
    float c = cosf(th), s = sinf(th);
    for (int i = 0; i < n; i++) {
        float x = in[i].x, y = in[i].y;
        out[i].x = c * x - s * y + tx;
        out[i].y = s * x + c * y + ty;
    }
}

static int cmp_float(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

/* numpy.quantile default (linear interpolation) */
static float quantile(float *v, int n, float q) {
    qsort(v, (size_t)n, sizeof(float), cmp_float);
    float pos = q * (float)(n - 1);
    int lo = (int)floorf(pos);
    int hi = lo + 1 < n ? lo + 1 : lo;
    return v[lo] + (pos - (float)lo) * (v[hi] - v[lo]);
}

/* Eigenvalues of a symmetric 3x3 by cyclic Jacobi rotations */
static void eig3_minmax(const double m[3][3], double *lmin, double *lmax) {
    double a[3][3];
    memcpy(a, m, sizeof(a));
    for (int sweep = 0; sweep < 20; sweep++) {
        double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if (off < 1e-30) break;
        for (int p = 0; p < 2; p++) {
            for (int q = p + 1; q < 3; q++) {
                if (fabs(a[p][q]) < 1e-300) continue;
                double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                double t = (theta >= 0 ? 1.0 : -1.0) / (fabs(theta) + sqrt(theta * theta + 1.0));
                double c = 1.0 / sqrt(t * t + 1.0), s = t * c;
                for (int k = 0; k < 3; k++) {
                    double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; k++) {
                    double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
            }
        }
    }
    *lmin = fmin(a[0][0], fmin(a[1][1], a[2][2]));
    *lmax = fmax(a[0][0], fmax(a[1][1], a[2][2]));
}

/* Solves the symmetric 3x3 system m x = r by Gaussian elimination */
static bool solve3(double m[3][3], double r[3], double x[3]) {
    double a[3][4];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) a[i][j] = m[i][j];
        a[i][3] = r[i];
    }
    for (int col = 0; col < 3; col++) {
        int piv = col;
        for (int i = col + 1; i < 3; i++) if (fabs(a[i][col]) > fabs(a[piv][col])) piv = i;
        if (fabs(a[piv][col]) < 1e-18) return false;
        if (piv != col) for (int j = 0; j < 4; j++) { double t = a[col][j]; a[col][j] = a[piv][j]; a[piv][j] = t; }
        for (int i = col + 1; i < 3; i++) {
            double f = a[i][col] / a[col][col];
            for (int j = col; j < 4; j++) a[i][j] -= f * a[col][j];
        }
    }
    for (int i = 2; i >= 0; i--) {
        double s = a[i][3];
        for (int j = i + 1; j < 3; j++) s -= a[i][j] * x[j];
        x[i] = s / a[i][i];
    }
    return true;
}

static bool reject(icp_line_info_t *info, icp_line_status_t status) {
    info->status = status;
    return false;
}

bool icp_point_to_line(const icp_points_t *Q, const icp_points_t *P,
                       const icp_line_config_t *config,
                       icp_point_t *translation, float *rotation,
                       icp_line_info_t *info) {
    icp_line_config_t cfg = config ? *config : icp_line_default_config();
    icp_line_info_t local;
    if (info == NULL) info = &local;
    memset(info, 0, sizeof(*info));
    info->rmse_m = NAN;
    translation->x = translation->y = 0.0f;
    *rotation = 0.0f;

    if (P->num > ICP_MAX_POINTS || Q->num > ICP_MAX_POINTS) return reject(info, ICP_LINE_TOO_FEW_POINTS);
    int ns = voxel_filter(P, s_src);
    int nt = voxel_filter(Q, s_dst);
    if (ns < cfg.min_points || nt < cfg.min_points) return reject(info, ICP_LINE_TOO_FEW_POINTS);
    if (degenerate(s_src, ns) || degenerate(s_dst, nt)) return reject(info, ICP_LINE_DEGENERATE);

    compute_normals(s_dst, nt);

    /* Estimate (th, tx, ty): moved = R(th) src + t; starts at identity */
    float th = 0.0f, tx = 0.0f, ty = 0.0f;
    bool converged = false;
    for (int it = 0; it < cfg.max_iterations; it++) {
        apply(s_src, ns, th, tx, ty, s_moved);
        nearest(s_moved, ns, s_dst, nt, s_match, s_dist);

        int nkeep = 0;
        for (int i = 0; i < ns; i++) {
            int j = s_match[i];
            if (s_dist[i] <= cfg.max_distance_m && s_normal_ok[j]) {
                s_err[i] = fabsf(s_normal[j].x * (s_moved[i].x - s_dst[j].x) +
                                 s_normal[j].y * (s_moved[i].y - s_dst[j].y));
                s_sorted[nkeep++] = s_err[i];
            } else {
                s_err[i] = -1.0f; /* not kept */
            }
        }
        if (nkeep < cfg.min_points) return reject(info, ICP_LINE_TOO_FEW_MATCHES);
        float trim = quantile(s_sorted, nkeep, TRIM_QUANTILE);

        /* Trim outlying normal errors, then keep the closest source point
         * per target point */
        for (int j = 0; j < nt; j++) s_best_src[j] = -1;
        for (int i = 0; i < ns; i++) {
            if (s_err[i] < 0.0f || s_err[i] > trim) continue;
            int j = s_match[i];
            if (s_best_src[j] < 0 || s_dist[i] < s_dist[s_best_src[j]]) s_best_src[j] = (int16_t)i;
        }

        double JtJ[3][3] = {{0}}, Jtr[3] = {0};
        int nsel = 0;
        for (int j = 0; j < nt; j++) {
            int i = s_best_src[j];
            if (i < 0) continue;
            double nx = s_normal[j].x, ny = s_normal[j].y;
            double ax = s_moved[i].x, ay = s_moved[i].y;
            double row[3] = {nx, ny, -nx * ay + ny * ax};
            double r = nx * (ax - s_dst[j].x) + ny * (ay - s_dst[j].y);
            for (int u = 0; u < 3; u++) {
                Jtr[u] += row[u] * r;
                for (int v = 0; v < 3; v++) JtJ[u][v] += row[u] * row[v];
            }
            nsel++;
        }
        if (nsel < cfg.min_points) return reject(info, ICP_LINE_TOO_FEW_MATCHES);

        double lmin, lmax;
        eig3_minmax(JtJ, &lmin, &lmax);
        if (lmax <= 0.0 || lmin / lmax < UNOBSERVABLE_RATIO) return reject(info, ICP_LINE_UNOBSERVABLE);

        double rhs[3] = {-Jtr[0], -Jtr[1], -Jtr[2]}, delta[3];
        if (!solve3(JtJ, rhs, delta)) return reject(info, ICP_LINE_UNOBSERVABLE);

        /* estimate = delta (+) estimate */
        float c = cosf((float)delta[2]), s = sinf((float)delta[2]);
        float ntx = c * tx - s * ty + (float)delta[0];
        float nty = s * tx + c * ty + (float)delta[1];
        tx = ntx;
        ty = nty;
        th += (float)delta[2];
        info->iterations = it + 1;
        info->last_step_m = (float)hypot(delta[0], delta[1]);
        info->last_step_rad = (float)fabs(delta[2]);
        if (hypot(delta[0], delta[1]) < CONVERGED_STEP && fabs(delta[2]) < CONVERGED_STEP) {
            converged = true;
            break;
        }
    }

    /* Final quality: forward and reverse overlap, point-to-line RMSE */
    apply(s_src, ns, th, tx, ty, s_moved);
    nearest(s_moved, ns, s_dst, nt, s_match, s_dist);
    int keep = 0;
    double sq = 0.0;
    for (int i = 0; i < ns; i++) {
        int j = s_match[i];
        if (s_dist[i] <= cfg.max_distance_m && s_normal_ok[j]) {
            double e = s_normal[j].x * (s_moved[i].x - s_dst[j].x) +
                       s_normal[j].y * (s_moved[i].y - s_dst[j].y);
            sq += e * e;
            keep++;
        }
    }
    static int16_t rev_idx[ICP_MAX_POINTS];
    static float rev_dist[ICP_MAX_POINTS];
    nearest(s_dst, nt, s_moved, ns, rev_idx, rev_dist);
    int rev = 0;
    for (int j = 0; j < nt; j++) if (rev_dist[j] <= cfg.max_distance_m) rev++;
    float fwd_overlap = (float)keep / (float)ns, rev_overlap = (float)rev / (float)nt;
    info->overlap = fwd_overlap < rev_overlap ? fwd_overlap : rev_overlap;
    info->rmse_m = keep > 0 ? (float)sqrt(sq / keep) : NAN;

    translation->x = tx;
    translation->y = ty;
    *rotation = th;

    if (!converged && !(info->last_step_m < SETTLED_STEP_M && info->last_step_rad < SETTLED_STEP_RAD)) {
        return reject(info, ICP_LINE_NOT_CONVERGED);
    }
    if (info->overlap < cfg.min_overlap) return reject(info, ICP_LINE_LOW_OVERLAP);
    if (!(info->rmse_m <= cfg.max_rmse_m)) return reject(info, ICP_LINE_HIGH_RMSE);
    info->status = ICP_LINE_ACCEPTED;
    return true;
}
