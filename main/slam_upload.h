#pragma once

/* Post-flight NanoSLAM upload, drone -> laptop over UDP (little-endian, packed).
 * Mirrored by laptop/nanoslam_receive.py; bump SLAM_UPLOAD_VERSION on change.
 *
 * One upload = SUMMARY, one NODE per graph node, CONSTRAINTS packets (up to
 * 64 constraints each), END. Each packet
 * carries its index within the upload so the laptop can detect gaps; the
 * whole upload is repeated CONFIG_NANOSLAM_UPLOAD_REPEATS times. */

#include <stdint.h>

#include "scan.h"

#define SLAM_UPLOAD_VERSION       3   /* 2: summary gains columns / tof_hz;
                                         3: constraints split over packets */
#define SLAM_PKT_SUMMARY          0x20
#define SLAM_PKT_NODE             0x21
#define SLAM_PKT_CONSTRAINTS      0x22
#define SLAM_PKT_END              0x23
#define SLAM_UPLOAD_MAX_CONSTRAINTS 64  /* per CONSTRAINTS packet */

typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;
    uint8_t  version;
    uint8_t  drone_id;
    uint8_t  reserved;
    uint32_t run_id;        /* ESP ms at the start of recording */
    uint16_t index;         /* packet index within this upload */
    uint16_t total;         /* packets in this upload */
} slam_upload_header_t;     /* 12 bytes */

typedef struct __attribute__((packed)) {
    slam_upload_header_t hdr;
    uint16_t node_count;
    uint16_t constraint_count;
    uint8_t  solved;        /* 0: last optimisation failed; optimised == stored poses */
    uint8_t  sensor_count;
    uint16_t record_interval_ms;
    float    sensor_yaw_rad[SCAN_NUM_DIRS]; /* body frame, clockwise from the nose */
    float    sensor_offset_m;
    uint32_t icp_ms;
    uint32_t solve_ms;
    uint16_t seq_accepted, seq_attempts;
    uint16_t loop_accepted, loop_attempts;
    uint16_t pos_resets;    /* PX4 position jumps removed from the odometry */
    uint16_t icp_method;    /* 0 = point-to-point (ETH/Duncan), 1 = point-to-line */
    /* RAM, sampled after every recorded pose and every optimisation */
    uint32_t internal_total;        /* internal 8-bit heap size, bytes */
    uint32_t internal_free_start;   /* free internal heap when the run started */
    uint32_t internal_free_min;     /* lowest free internal heap during the run */
    uint32_t internal_free_min_boot;/* ESP-IDF low-water mark since boot */
    uint32_t internal_largest_min;  /* smallest "largest free internal block" seen */
    uint32_t psram_total;           /* 0 if no PSRAM */
    uint32_t psram_free_start;
    uint32_t psram_free_min;
    uint32_t slam_static_bytes;     /* NanoSLAM .bss: solver + ICP scratch */
    uint32_t slam_graph_bytes;      /* heap (PSRAM if present): pose/scan storage + solver matrices */
    uint32_t slam_stack_size, slam_stack_free_min;   /* recorder task, bytes */
    uint32_t opt_stack_size, opt_stack_free_min;     /* optimiser task, bytes */
    uint8_t  columns;       /* ToF columns per sensor in each frame: 8 (8x8) or 4 (4x4) */
    uint8_t  tof_hz;
    uint8_t  weights;       /* 0 = ETH (also all firmware before this field), 1 = per-axis + raw odometry */
    uint8_t  reserved2;
} slam_upload_summary_t;

typedef struct __attribute__((packed)) {
    float   dx, dy, dyaw;
    int16_t dists[SCAN_NUM_DIRS][SCAN_NUM_COLS]; /* only the first `columns` used */
} slam_upload_frame_t;      /* 140 bytes */

typedef struct __attribute__((packed)) {
    slam_upload_header_t hdr;
    uint16_t node;
    uint16_t reserved;
    float    raw[3];        /* jump-free PX4 odometry, NED: north, east, yaw (cw) */
    float    optimised[3];  /* graph pose after LS-SLAM, same frame */
    slam_upload_frame_t frames[SCAN_NUM_FRAMES];
} slam_upload_node_t;       /* 740 bytes */

typedef struct __attribute__((packed)) {
    int16_t from, to;       /* from = newer node, to = older node */
    uint8_t kind;           /* 0 = sequential ICP, 1 = loop closure */
    uint8_t reserved;
    float   z[3];           /* pose of `to` in the frame of `from` */
} slam_upload_constraint_t; /* 18 bytes */

typedef struct __attribute__((packed)) {
    slam_upload_header_t hdr;
    uint16_t count;
    uint16_t first;         /* index of items[0] among all constraints (v3) */
    slam_upload_constraint_t items[SLAM_UPLOAD_MAX_CONSTRAINTS];
} slam_upload_constraints_t; /* up to 1168 bytes */

typedef struct __attribute__((packed)) {
    slam_upload_header_t hdr;
    uint16_t node_count;
    uint16_t reserved;
} slam_upload_end_t;

_Static_assert(sizeof(slam_upload_header_t) == 12, "upload header layout");
_Static_assert(sizeof(slam_upload_frame_t) == 140, "upload frame layout");
_Static_assert(sizeof(slam_upload_node_t) == 740, "upload node layout");
_Static_assert(sizeof(slam_upload_summary_t) == 136, "upload summary layout");
_Static_assert(sizeof(slam_upload_constraint_t) == 18, "upload constraint layout");
_Static_assert(sizeof(slam_upload_constraints_t) <= 1472, "constraints fit one datagram");
