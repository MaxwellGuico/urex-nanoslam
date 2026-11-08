/*
 * Point-to-line ICP, ported from icp() in laptop/offline_slam.py so the drone
 * and the laptop pipeline match scans the same way.
 *
 * Point-to-point ICP (icp.c, ETH/Duncan) pulls each point onto the nearest
 * old point. ToF zones hit different spots on a wall from each pose, so that
 * drags scans sideways along walls (1-2 deg / few cm bias). Point-to-line
 * fits a local line through the reference scan and minimises only each
 * point's distance to that line, measured along the line's normal.
 *
 * Both clouds are in the same (world) frame; the result (rotation, then
 * translation) maps P onto Q, the same convention as icp().
 */

#ifndef ICP_LINE_H
#define ICP_LINE_H

#include <stdbool.h>

#include "icp.h"

typedef struct {
    float max_distance_m;  /* correspondence gate (offline: 0.25) */
    float min_overlap;     /* min of forward and reverse overlap (0.5) */
    float max_rmse_m;      /* point-to-line RMS error after alignment (0.08) */
    int min_points;        /* per cloud after 3 cm voxel filter (12) */
    int max_iterations;    /* (40) */
} icp_line_config_t;

typedef enum {
    ICP_LINE_ACCEPTED = 0,
    ICP_LINE_TOO_FEW_POINTS,
    ICP_LINE_DEGENERATE,           /* one straight wall: slide along it unobservable */
    ICP_LINE_TOO_FEW_MATCHES,
    ICP_LINE_UNOBSERVABLE,
    ICP_LINE_NOT_CONVERGED,
    ICP_LINE_LOW_OVERLAP,
    ICP_LINE_HIGH_RMSE,
} icp_line_status_t;

typedef struct {
    icp_line_status_t status;
    int iterations;
    float overlap;
    float rmse_m;
    float last_step_m;     /* size of the final Gauss-Newton update */
    float last_step_rad;
} icp_line_info_t;

icp_line_config_t icp_line_default_config(void);

/* Aligns P onto Q. Writes the transform and returns true only if every
 * quality check passed (info->status == ICP_LINE_ACCEPTED). Not re-entrant
 * (static buffers). Clouds larger than ICP_MAX_POINTS are rejected. */
bool icp_point_to_line(const icp_points_t *Q, const icp_points_t *P,
                       const icp_line_config_t *config,
                       icp_point_t *translation, float *rotation,
                       icp_line_info_t *info);

const char *icp_line_status_name(icp_line_status_t status);

#endif /* ICP_LINE_H */
