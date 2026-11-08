/*
 * Bridge between the flight tasks and the NanoSLAM component. Adapted from
 * slam_bridge.c and scan_acquire() in Duncan Lau's ESP32-S3 NanoSLAM port
 * (itself derived from ETH-PBL/NanoSLAM, GPL-3.0). Differences: recording is
 * driven by its own task from arm to disarm instead of the mission loop, the
 * in-flight optimiser is one persistent task, the correction is not fed to
 * navigation, and the result is uploaded to the laptop instead of printed.
 */

#include "slam_bridge.h"
#include "sdkconfig.h"

/* Built only with NanoSLAM; main.c references it under the same condition. */
#if CONFIG_NANOSLAM_ENABLED

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#include "mavlink_task.h"
#include "scan.h"
#include "slam.h"
#include "slam_graph.h"
#include "slam_upload.h"
#include "tof_task.h"

static const char *TAG = "slam_bridge";

#ifndef CONFIG_NANOSLAM_RECORD_DISARMED
#define CONFIG_NANOSLAM_RECORD_DISARMED 0
#endif
#ifndef CONFIG_NANOSLAM_ICP_POINT_TO_LINE
#define CONFIG_NANOSLAM_ICP_POINT_TO_LINE 0
#endif
#ifndef CONFIG_NANOSLAM_ETH_WEIGHTS
#define CONFIG_NANOSLAM_ETH_WEIGHTS 0
#endif

#define SLAM_PI              3.14159265358979323846f
#define SLAM_POSE_MAX_AGE_MS 250U
#define SLAM_OPT_TASK_STACK  16384 /* ICP + ls_slam; peak reported in the RAM summary */
#define SLAM_MAX_SHIFT_M     ((float)CONFIG_NANOSLAM_MAX_SHIFT_CM / 100.0f)

/* Held for a whole pose record (incl. the 400 ms scan) and for commits, so a
 * commit never interleaves with a half-recorded pose. */
static SemaphoreHandle_t s_bridge_mutex;
static TaskHandle_t s_opt_task;
static uint16_t s_node_idx;
static uint32_t s_run_id;
/* Odometry poses as recorded, uploaded so the laptop can show what SLAM
 * changed, and used to re-anchor poses on every commit. */
static float s_raw_poses[GRAPH_MAX_POSES][3];
/* Optimiser output; one optimisation at a time (s_opt_running) */
static float s_opt_poses[GRAPH_MAX_POSES][3];
static volatile bool s_opt_running;
static slam_report_t s_last_report;
static float s_sensor_yaw[SCAN_NUM_DIRS];
static TaskHandle_t s_slam_task;

/* RAM use over the current run, sampled after every pose and optimisation */
typedef struct {
    uint32_t internal_free_start, internal_free_min, internal_largest_min;
    uint32_t psram_free_start, psram_free_min;
    uint32_t slam_stack_free_min, opt_stack_free_min;
} ram_stats_t;
static ram_stats_t s_ram;
static portMUX_TYPE s_ram_lock = portMUX_INITIALIZER_UNLOCKED;

#define INTERNAL_CAPS (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#define PSRAM_CAPS    (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

static uint32_t stack_free_bytes(TaskHandle_t task) {
    /* ESP-IDF reports the high-water mark in bytes */
    return task ? (uint32_t)uxTaskGetStackHighWaterMark(task) : 0;
}

static void ram_reset(void) {
    ram_stats_t r = {
        .internal_free_start = heap_caps_get_free_size(INTERNAL_CAPS),
        .internal_largest_min = heap_caps_get_largest_free_block(INTERNAL_CAPS),
        .psram_free_start = heap_caps_get_free_size(PSRAM_CAPS),
        .slam_stack_free_min = stack_free_bytes(s_slam_task),
        .opt_stack_free_min = stack_free_bytes(s_opt_task),
    };
    r.internal_free_min = r.internal_free_start;
    r.psram_free_min = r.psram_free_start;
    taskENTER_CRITICAL(&s_ram_lock);
    s_ram = r;
    taskEXIT_CRITICAL(&s_ram_lock);
}

static void ram_sample(const char *when) {
    uint32_t internal = heap_caps_get_free_size(INTERNAL_CAPS);
    uint32_t largest = heap_caps_get_largest_free_block(INTERNAL_CAPS);
    uint32_t psram = heap_caps_get_free_size(PSRAM_CAPS);
    uint32_t slam_stack = stack_free_bytes(s_slam_task);
    uint32_t opt_stack = stack_free_bytes(s_opt_task);
    taskENTER_CRITICAL(&s_ram_lock);
    if (internal < s_ram.internal_free_min) s_ram.internal_free_min = internal;
    if (largest < s_ram.internal_largest_min) s_ram.internal_largest_min = largest;
    if (psram < s_ram.psram_free_min) s_ram.psram_free_min = psram;
    if (slam_stack < s_ram.slam_stack_free_min) s_ram.slam_stack_free_min = slam_stack;
    if (opt_stack < s_ram.opt_stack_free_min) s_ram.opt_stack_free_min = opt_stack;
    taskEXIT_CRITICAL(&s_ram_lock);
    ESP_LOGD(TAG, "RAM after %s: internal free %lu (largest %lu), PSRAM free %lu, "
                  "stack free slam %lu / opt %lu",
             when, (unsigned long)internal, (unsigned long)largest, (unsigned long)psram,
             (unsigned long)slam_stack, (unsigned long)opt_stack);
}

static void ram_log_summary(void) {
    taskENTER_CRITICAL(&s_ram_lock);
    ram_stats_t r = s_ram;
    taskEXIT_CRITICAL(&s_ram_lock);
    ESP_LOGI(TAG, "RAM this run: internal free %lu -> min %lu of %lu (boot min %lu, largest block min %lu); "
                  "PSRAM free %lu -> min %lu of %lu; NanoSLAM static %u B + heap %u B; "
                  "stack free min slam %lu/%d, opt %lu/%d",
             (unsigned long)r.internal_free_start, (unsigned long)r.internal_free_min,
             (unsigned long)heap_caps_get_total_size(INTERNAL_CAPS),
             (unsigned long)heap_caps_get_minimum_free_size(INTERNAL_CAPS),
             (unsigned long)r.internal_largest_min,
             (unsigned long)r.psram_free_start, (unsigned long)r.psram_free_min,
             (unsigned long)heap_caps_get_total_size(PSRAM_CAPS),
             (unsigned)slam_static_bytes(), (unsigned)(slam_graph_storage_bytes() + ls_slam_heap_bytes()),
             (unsigned long)r.slam_stack_free_min, SLAM_TASK_STACK,
             (unsigned long)r.opt_stack_free_min, SLAM_OPT_TASK_STACK);
}

/* ---------------------------------------------------------------------------
 * SE(2) helpers: x = north, y = east, yaw clockwise (NED)
 * --------------------------------------------------------------------------- */
typedef struct { float x, y, yaw; } se2_t;

static se2_t se2_compose(se2_t a, se2_t b) {
    float c = cosf(a.yaw), s = sinf(a.yaw);
    return (se2_t){a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw};
}

static se2_t se2_inverse(se2_t a) {
    float c = cosf(a.yaw), s = sinf(a.yaw);
    return (se2_t){-c * a.x - s * a.y, s * a.x - c * a.y, -a.yaw};
}

/* map = s_corr (+) odom, guarded by a short critical section rather than
 * s_bridge_mutex (held for 400 ms scans). */
static se2_t s_corr;
static portMUX_TYPE s_corr_lock = portMUX_INITIALIZER_UNLOCKED;

static se2_t get_corr(void) {
    taskENTER_CRITICAL(&s_corr_lock);
    se2_t c = s_corr;
    taskEXIT_CRITICAL(&s_corr_lock);
    return c;
}

static void set_corr(se2_t c) {
    taskENTER_CRITICAL(&s_corr_lock);
    s_corr = c;
    taskEXIT_CRITICAL(&s_corr_lock);
}

void slam_bridge_odom_to_map(float odom_x, float odom_y, float *map_x, float *map_y) {
    se2_t m = se2_compose(get_corr(), (se2_t){odom_x, odom_y, 0.0f});
    *map_x = m.x;
    *map_y = m.y;
}

void slam_bridge_map_to_odom(float map_x, float map_y, float *odom_x, float *odom_y) {
    se2_t o = se2_compose(se2_inverse(get_corr()), (se2_t){map_x, map_y, 0.0f});
    *odom_x = o.x;
    *odom_y = o.y;
}

size_t slam_bridge_get_pose_count(void) { return slam_graph_pose_count(); }
bool slam_bridge_is_optimizing(void) { return s_opt_running; }

/* ---------------------------------------------------------------------------
 * Scan acquisition: SCAN_NUM_FRAMES ring snapshots over SCAN_TIME_MS
 * --------------------------------------------------------------------------- */
static void scan_acquire(scan_t *scan) {
    drone_state_t st0 = mavlink_get_state();
    float x0 = st0.slam_x, y0 = st0.slam_y, yaw0 = st0.heading;

    /* Per-scan drop counts, in columns (8 per sensor frame) */
    int n_stale = 0, n_spin = 0, n_rows = 0;

    TickType_t prev_wake = xTaskGetTickCount();
    for (int i = 0; i < SCAN_NUM_FRAMES; i++) {
        drone_state_t st = mavlink_get_state();
        scan->frames[i].dx = st.slam_x - x0;
        scan->frames[i].dy = st.slam_y - y0;
        scan->frames[i].dyaw = st.heading - yaw0;

        tof_scan_t raw = tof_get_scan();
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        for (int dir = 0; dir < SCAN_NUM_DIRS; dir++) {
            const tof_frame_t *frame = &raw.frame[dir];
            if (!raw.sensor_ok[dir] || !frame->valid) {
                scan_invalidate_dists(scan->frames[i].dists[dir]);
                continue;
            }
            /* The frame was measured before this pose sample; an old frame
             * would land in the wrong place. */
            uint32_t age_ms = now_ms - frame->timestamp_ms;
            if (age_ms > SCAN_MAX_FRAME_AGE_MS) {
                scan_invalidate_dists(scan->frames[i].dists[dir]);
                n_stale += TOF_NATIVE_SIDE;
                continue;
            }
            if (fabsf(st.yawspeed) * (float)age_ms / 1000.0f > SCAN_MAX_YAW_ERR_RAD) {
                scan_invalidate_dists(scan->frames[i].dists[dir]);
                n_spin += TOF_NATIVE_SIDE;
                continue;
            }
            n_rows += scan_reduce_frame(frame->distance_mm, frame->target_status,
                                        TOF_NATIVE_SIDE, scan->frames[i].dists[dir]);
        }
        vTaskDelayUntil(&prev_wake, pdMS_TO_TICKS(SCAN_TIME_MS / SCAN_NUM_FRAMES));
    }

    ESP_LOGI(TAG, "Scan for pose %u: kept %d / stale %d / spin %d / rows %d",
             (unsigned)scan->pose, scan_count_valid(scan), n_stale, n_spin, n_rows);
}

/* ---------------------------------------------------------------------------
 * Commit
 * --------------------------------------------------------------------------- */

/* Applies s_opt_poses[0..n-1] and moves the correction so the newest optimised
 * pose becomes the anchor. Poses recorded while the optimiser ran
 * (n..s_node_idx-1) are re-anchored from their raw odometry. Rejects the
 * result if the newest pose would move more than max_shift_m.
 * Caller holds s_bridge_mutex. */
static bool commit_locked(size_t n, float max_shift_m) {
    slam_pose_t last;
    if (n < 2 || !slam_graph_get_pose(n - 1, &last)) return false;

    float shift = hypotf(s_opt_poses[n - 1][0] - last.x, s_opt_poses[n - 1][1] - last.y);
    if (shift > max_shift_m) {
        ESP_LOGW(TAG, "SLAM result rejected: would move current estimate %.2f m (max %.2f)",
                 shift, max_shift_m);
        return false;
    }

    slam_commit_poses(s_opt_poses, n);

    se2_t anchor_map = {s_opt_poses[n - 1][0], s_opt_poses[n - 1][1], s_opt_poses[n - 1][2]};
    se2_t anchor_odom = {s_raw_poses[n - 1][0], s_raw_poses[n - 1][1], s_raw_poses[n - 1][2]};
    se2_t corr = se2_compose(anchor_map, se2_inverse(anchor_odom));
    set_corr(corr);

    for (uint16_t i = (uint16_t)n; i < s_node_idx; i++) {
        se2_t m = se2_compose(corr, (se2_t){s_raw_poses[i][0], s_raw_poses[i][1], s_raw_poses[i][2]});
        slam_pose_t p = {m.x, m.y, m.yaw};
        slam_graph_set_pose(i, &p);
    }

    ESP_LOGI(TAG, "SLAM committed %u poses (+%u re-anchored): estimate moved %.2f m, "
                  "correction N%+.2f E%+.2f yaw%+.3f",
             (unsigned)n, (unsigned)(s_node_idx - n), shift, corr.x, corr.y, corr.yaw);
    return true;
}

static void optimise_and_commit(float max_shift_m) {
    uint64_t t0 = esp_timer_get_time();
    slam_report_t rep;
    size_t n = slam_optimize(s_opt_poses, &rep);
    s_last_report = rep;
    ram_sample("optimisation");
    if (n >= 2 && rep.solved) {
        xSemaphoreTake(s_bridge_mutex, portMAX_DELAY);
        commit_locked(n, max_shift_m);
        xSemaphoreGive(s_bridge_mutex);
    }
    ESP_LOGI(TAG, "SLAM pass over %u poses finished in %llu ms (seq %u/%u, loop %u/%u%s)",
             (unsigned)n, (unsigned long long)((esp_timer_get_time() - t0) / 1000),
             rep.sequential_accepted, rep.sequential_attempts,
             rep.loop_accepted, rep.loop_attempts, rep.solved ? "" : ", solve rejected");
}

/* ---------------------------------------------------------------------------
 * In-flight background optimisation (core 0, priority 1: below MAVLink and
 * ToF, so sensing and the flight link always win)
 * --------------------------------------------------------------------------- */
static void slam_opt_task(void *arg) {
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        optimise_and_commit(SLAM_MAX_SHIFT_M);
        s_opt_running = false;
    }
}

static void maybe_trigger_inflight(void) {
#if CONFIG_NANOSLAM_INFLIGHT_EVERY_N > 0
    if (s_node_idx % CONFIG_NANOSLAM_INFLIGHT_EVERY_N != 0 || s_opt_running) return;
    s_opt_running = true;
    xTaskNotifyGive(s_opt_task);
#endif
}

static void wait_for_optimiser(void) {
    while (s_opt_running) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ---------------------------------------------------------------------------
 * Recording
 * --------------------------------------------------------------------------- */
static void reset_run(void) {
    wait_for_optimiser();
    xSemaphoreTake(s_bridge_mutex, portMAX_DELAY);
    slam_graph_reset();
    s_node_idx = 0;
    s_run_id = (uint32_t)(esp_timer_get_time() / 1000);
    memset(&s_last_report, 0, sizeof(s_last_report));
    set_corr((se2_t){0.0f, 0.0f, 0.0f});
    xSemaphoreGive(s_bridge_mutex);
    ram_reset();
    ESP_LOGI(TAG, "New SLAM run %lu", (unsigned long)s_run_id);
}

static bool record_pose(void) {
    static scan_t sc;
    drone_state_t st = mavlink_get_state();
    /* Jump-free odometry: PX4 estimator resets can shift whole segments */
    float x = st.slam_x, y = st.slam_y, yaw = st.heading;

    xSemaphoreTake(s_bridge_mutex, portMAX_DELAY);
    if (s_node_idx >= GRAPH_MAX_POSES) {
        xSemaphoreGive(s_bridge_mutex);
        return false;
    }
    /* Store in the map frame so the graph stays continuous with poses that
     * earlier optimisations already corrected. */
    se2_t m = se2_compose(get_corr(), (se2_t){x, y, yaw});
    sc.pose = s_node_idx;
    scan_acquire(&sc);

    slam_pose_t p = {m.x, m.y, m.yaw};
    const slam_pose_t odom = {x, y, yaw};
    int index = slam_graph_add(&p, &odom, &sc);
    if (index >= 0) {
        s_raw_poses[index][0] = x;
        s_raw_poses[index][1] = y;
        s_raw_poses[index][2] = yaw;
        s_node_idx++;
        ESP_LOGI(TAG, "Pose %d/%d: N=%.2f E=%.2f yaw=%.1fdeg", index + 1, GRAPH_MAX_POSES,
                 p.x, p.y, p.yaw * 180.0f / SLAM_PI);
        maybe_trigger_inflight();
    }
    xSemaphoreGive(s_bridge_mutex);
    ram_sample("pose");
    return index >= 0;
}

/* ---------------------------------------------------------------------------
 * Upload to the laptop
 * --------------------------------------------------------------------------- */
static void fill_header(slam_upload_header_t *h, uint8_t type, uint16_t index, uint16_t total) {
    h->pkt_type = type;
    h->version = SLAM_UPLOAD_VERSION;
    h->drone_id = CONFIG_DRONE_ID;
    h->reserved = 0;
    h->run_id = s_run_id;
    h->index = index;
    h->total = total;
}

static void send_packet(int sock, const void *data, size_t len) {
    /* Pace the burst so the Wi-Fi TX queue does not drop datagrams */
    for (int attempt = 0; attempt < 5; attempt++) {
        if (send(sock, data, len, 0) >= 0) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(4));
}

static void upload_result(void) {
    static slam_upload_node_t node;
    static slam_upload_constraints_t cons_pkt;
    static scan_t sc;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGW(TAG, "SLAM upload: no socket");
        return;
    }
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_NANOSLAM_UPLOAD_PORT),
    };
    inet_aton(CONFIG_HOST_IPV4_ADDR, &dest.sin_addr);
    connect(sock, (struct sockaddr *)&dest, sizeof(dest));

    const size_t n = slam_graph_pose_count();
    const constraint *cons = NULL;
    const uint8_t *kinds = NULL;
    const size_t nc = slam_last_constraints(&cons, &kinds);
    const size_t chunks = nc == 0 ? 1 : (nc + SLAM_UPLOAD_MAX_CONSTRAINTS - 1) / SLAM_UPLOAD_MAX_CONSTRAINTS;
    const uint16_t total = (uint16_t)(n + 2 + chunks);

    slam_upload_summary_t summary;
    memset(&summary, 0, sizeof(summary));
    fill_header(&summary.hdr, SLAM_PKT_SUMMARY, 0, total);
    summary.node_count = (uint16_t)n;
    summary.constraint_count = (uint16_t)nc;
    summary.solved = s_last_report.solved;
    summary.sensor_count = SCAN_NUM_DIRS;
    summary.record_interval_ms = CONFIG_NANOSLAM_RECORD_INTERVAL_MS;
    memcpy(summary.sensor_yaw_rad, s_sensor_yaw, sizeof(summary.sensor_yaw_rad));
    summary.sensor_offset_m = (float)CONFIG_NANOSLAM_SENSOR_OFFSET_MM / 1000.0f;
    summary.icp_ms = s_last_report.icp_ms;
    summary.solve_ms = s_last_report.solve_ms;
    summary.seq_accepted = s_last_report.sequential_accepted;
    summary.seq_attempts = s_last_report.sequential_attempts;
    summary.loop_accepted = s_last_report.loop_accepted;
    summary.loop_attempts = s_last_report.loop_attempts;
    summary.pos_resets = mavlink_get_state().pos_resets;
    summary.icp_method = (uint16_t)slam_get_icp_method();
    summary.columns = (uint8_t)scan_get_columns();
    summary.tof_hz = (uint8_t)TOF_RANGING_FREQ_HZ;
    summary.weights = CONFIG_NANOSLAM_ETH_WEIGHTS ? 0 : 1;
    taskENTER_CRITICAL(&s_ram_lock);
    ram_stats_t ram = s_ram;
    taskEXIT_CRITICAL(&s_ram_lock);
    summary.internal_total = heap_caps_get_total_size(INTERNAL_CAPS);
    summary.internal_free_start = ram.internal_free_start;
    summary.internal_free_min = ram.internal_free_min;
    summary.internal_free_min_boot = heap_caps_get_minimum_free_size(INTERNAL_CAPS);
    summary.internal_largest_min = ram.internal_largest_min;
    summary.psram_total = heap_caps_get_total_size(PSRAM_CAPS);
    summary.psram_free_start = ram.psram_free_start;
    summary.psram_free_min = ram.psram_free_min;
    summary.slam_static_bytes = slam_static_bytes();
    summary.slam_graph_bytes = slam_graph_storage_bytes() + ls_slam_heap_bytes();
    summary.slam_stack_size = SLAM_TASK_STACK;
    summary.slam_stack_free_min = ram.slam_stack_free_min;
    summary.opt_stack_size = SLAM_OPT_TASK_STACK;
    summary.opt_stack_free_min = ram.opt_stack_free_min;

    slam_upload_end_t end;
    memset(&end, 0, sizeof(end));
    fill_header(&end.hdr, SLAM_PKT_END, (uint16_t)(total - 1), total);
    end.node_count = (uint16_t)n;

    for (int round = 0; round < CONFIG_NANOSLAM_UPLOAD_REPEATS; round++) {
        send_packet(sock, &summary, sizeof(summary));
        for (size_t i = 0; i < n; i++) {
            slam_pose_t p;
            if (!slam_graph_get_pose(i, &p) || !slam_graph_get_scan(i, &sc)) continue;
            memset(&node, 0, sizeof(node));
            fill_header(&node.hdr, SLAM_PKT_NODE, (uint16_t)(i + 1), total);
            node.node = (uint16_t)i;
            memcpy(node.raw, s_raw_poses[i], sizeof(node.raw));
            node.optimised[0] = p.x;
            node.optimised[1] = p.y;
            node.optimised[2] = p.yaw;
            for (int f = 0; f < SCAN_NUM_FRAMES; f++) {
                node.frames[f].dx = sc.frames[f].dx;
                node.frames[f].dy = sc.frames[f].dy;
                node.frames[f].dyaw = sc.frames[f].dyaw;
                memcpy(node.frames[f].dists, sc.frames[f].dists, sizeof(node.frames[f].dists));
            }
            send_packet(sock, &node, sizeof(node));
        }
        for (size_t c = 0; c < chunks; c++) {
            const size_t first = c * SLAM_UPLOAD_MAX_CONSTRAINTS;
            size_t count = nc - first;
            if (nc == 0) count = 0;
            if (count > SLAM_UPLOAD_MAX_CONSTRAINTS) count = SLAM_UPLOAD_MAX_CONSTRAINTS;
            memset(&cons_pkt, 0, sizeof(cons_pkt));
            fill_header(&cons_pkt.hdr, SLAM_PKT_CONSTRAINTS, (uint16_t)(n + 1 + c), total);
            cons_pkt.count = (uint16_t)count;
            cons_pkt.first = (uint16_t)first;
            for (size_t k = 0; k < count; k++) {
                cons_pkt.items[k].from = cons[first + k].fromIdx;
                cons_pkt.items[k].to = cons[first + k].toIdx;
                cons_pkt.items[k].kind = kinds[first + k];
                memcpy(cons_pkt.items[k].z, cons[first + k].z, sizeof(cons_pkt.items[k].z));
            }
            send_packet(sock, &cons_pkt, offsetof(slam_upload_constraints_t, items) +
                                             count * sizeof(slam_upload_constraint_t));
        }
        send_packet(sock, &end, sizeof(end));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    close(sock);
    ESP_LOGI(TAG, "SLAM run %lu uploaded to %s:%d (%u nodes, %u constraints, x%d)",
             (unsigned long)s_run_id, CONFIG_HOST_IPV4_ADDR, CONFIG_NANOSLAM_UPLOAD_PORT,
             (unsigned)n, (unsigned)nc, CONFIG_NANOSLAM_UPLOAD_REPEATS);
}

static void finish_run(void) {
    if (slam_graph_pose_count() < 2) {
        ESP_LOGW(TAG, "SLAM run %lu has %u poses; nothing to optimise",
                 (unsigned long)s_run_id, (unsigned)slam_graph_pose_count());
        return;
    }
    wait_for_optimiser();
    s_opt_running = true;
    ESP_LOGI(TAG, "Post-flight LS-SLAM optimisation...");
    optimise_and_commit(INFINITY); /* on the ground: no live estimate to protect */
    s_opt_running = false;
    ram_log_summary();
    upload_result();
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
void slam_bridge_init(void) {
    s_bridge_mutex = xSemaphoreCreateMutex();
    configASSERT(s_bridge_mutex != NULL);
    configASSERT(slam_graph_init());
    for (int k = 0; k < SCAN_NUM_DIRS; k++) {
        s_sensor_yaw[k] = tof_sensor_angle_deg((uint8_t)k) * SLAM_PI / 180.0f;
    }
    scan_set_geometry(s_sensor_yaw, (float)CONFIG_NANOSLAM_SENSOR_OFFSET_MM / 1000.0f);
    scan_set_columns(TOF_NATIVE_SIDE);
    slam_set_icp_method(CONFIG_NANOSLAM_ICP_POINT_TO_LINE ? SLAM_ICP_POINT_TO_LINE
                                                         : SLAM_ICP_POINT_TO_POINT);
    const ls_slam_weights_t weights = CONFIG_NANOSLAM_ETH_WEIGHTS
                                          ? (ls_slam_weights_t)SLAM_WEIGHTS_ETH
                                          : (ls_slam_weights_t)SLAM_WEIGHTS_DEFAULT;
    slam_set_weights(&weights);
    s_run_id = (uint32_t)(esp_timer_get_time() / 1000);
    BaseType_t ok = xTaskCreatePinnedToCore(slam_opt_task, "slam_opt", SLAM_OPT_TASK_STACK,
                                            NULL, SLAM_TASK_PRIORITY, &s_opt_task, SLAM_TASK_CORE);
    configASSERT(ok == pdPASS);
    ram_reset();
}

void slam_task(void *arg) {
    (void)arg;
    s_slam_task = xTaskGetCurrentTaskHandle();
    ram_reset();
    bool was_armed = false;
    bool finished = false; /* disarmed-recording run already uploaded */
    uint32_t last_record_ms = 0;

    if (TOF_NATIVE_SIDE != 8 && TOF_NATIVE_SIDE != 4) {
        ESP_LOGW(TAG, "NanoSLAM supports the 8x8 and 4x4 ToF profiles (have %dx%d); SLAM disabled",
                 TOF_NATIVE_SIDE, TOF_NATIVE_SIDE);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "NanoSLAM ready: %s ICP, %s weights, ToF %dx%d @ %d Hz, pose every %d ms, "
                  "in-flight opt every %d poses, upload to %s:%d",
             CONFIG_NANOSLAM_ICP_POINT_TO_LINE ? "point-to-line" : "point-to-point",
             CONFIG_NANOSLAM_ETH_WEIGHTS ? "ETH" : "per-axis",
             TOF_NATIVE_SIDE, TOF_NATIVE_SIDE, TOF_RANGING_FREQ_HZ,
             CONFIG_NANOSLAM_RECORD_INTERVAL_MS, CONFIG_NANOSLAM_INFLIGHT_EVERY_N,
             CONFIG_HOST_IPV4_ADDR, CONFIG_NANOSLAM_UPLOAD_PORT);

    for (;;) {
        const drone_state_t st = mavlink_get_state();
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        const bool fresh = st.last_pos_ms != 0 && st.last_att_ms != 0 &&
                           now_ms - st.last_pos_ms <= SLAM_POSE_MAX_AGE_MS &&
                           now_ms - st.last_att_ms <= SLAM_POSE_MAX_AGE_MS &&
                           isfinite(st.slam_x) && isfinite(st.slam_y) && isfinite(st.heading);

        if (st.armed && !was_armed) {
            reset_run();
            finished = false;
            last_record_ms = 0;
        } else if (!st.armed && was_armed) {
            finish_run();
            finished = true;
        }
        was_armed = st.armed;

        const bool recording = st.armed || (CONFIG_NANOSLAM_RECORD_DISARMED && !finished);
        if (recording && fresh && s_node_idx < GRAPH_MAX_POSES &&
            (last_record_ms == 0 ||
             now_ms - last_record_ms >= (uint32_t)CONFIG_NANOSLAM_RECORD_INTERVAL_MS)) {
            last_record_ms = now_ms;
            record_pose();
            if (s_node_idx >= GRAPH_MAX_POSES) {
                ESP_LOGW(TAG, "SLAM pose graph full (%d poses)", GRAPH_MAX_POSES);
                if (!st.armed) {
                    finish_run(); /* disarmed bench recording */
                    finished = true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#endif /* CONFIG_NANOSLAM_ENABLED */
