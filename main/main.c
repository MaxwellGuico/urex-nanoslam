#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "mavlink_task.h"
#include "slam_bridge.h"
#include "tof_task.h"
#include "nav_task.h"
#include "at_detect.h"
#include "wifi_task.h"
#include "odom.h"
#include "experiment_task.h"

#include <math.h>

static const char *TAG = "mission";

#define PRECISION_APPROACH_S     10     /* max seconds to fly to tag position     */

/* ---------------------------------------------------------------------------
 * Test parameters — adjust before flight
 * --------------------------------------------------------------------------- */
#define CRUISE_ALT_M        0.5f    /* target altitude above takeoff (m)        */
#define GOAL_OFFSET_X_M     2.0f    /* goal North offset from takeoff (m)       */
#define GOAL_OFFSET_Y_M     1.6f    /* goal East  offset from takeoff (m)       */
#define TAKEOFF_TIMEOUT_S   10      /* max seconds to wait for altitude (s)     */
#define LAND_TIMEOUT_S      20      /* max seconds to wait for disarm (s)       */
#define ALT_TOLERANCE_M     0.15f   /* altitude band considered "at altitude"   */

/* ---------------------------------------------------------------------------
 * Mission task — runs once, handles full flight sequence
 *
 * Sequencing contract with nav_task:
 *   1. nav_task is idle (no goal) — mission_task owns the MAVLink setpoint
 *   2. nav_set_goal_map() is called — nav_task takes over setpoints
 *   3. nav_cancel() restores ownership to mission_task for landing
 * --------------------------------------------------------------------------- */
#if !CONFIG_BENCH_MODE && !CONFIG_EXPERIMENT_MISSION_ENABLED
static void mission_task(void *arg)
{
    bool  nav_goal_active = false;
    float target_z   = -(CRUISE_ALT_M);   /* NED: negative = above ground */
    float takeoff_x  = 0.0f;
    float takeoff_y  = 0.0f;
    float takeoff_heading = 0.0f;

    /* ------------------------------------------------------------------ */
    /* Phase 1: Wait for valid telemetry                                   */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Waiting for telemetry...");
    while (1) {
        drone_state_t st = mavlink_get_state();
        if (mavlink_position_valid() && st.last_hb_ms != 0) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Telemetry valid");

    /* ------------------------------------------------------------------ */
    /* Phase 2: Pre-stream hold setpoint                                   */
    /* PX4 requires setpoints to already be streaming before it will       */
    /* accept an OFFBOARD mode switch.  Stream for 2 s.                    */
    /* ------------------------------------------------------------------ */
    mavlink_set_hold();
    ESP_LOGI(TAG, "Pre-streaming hold setpoint for 2 s...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* ------------------------------------------------------------------ */
    /* Phase 2b: Wait for CMD_START from laptop                           */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Waiting for CMD_START from laptop...");
    while (!wifi_start_requested()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    wifi_clear_start_request();
    ESP_LOGI(TAG, "CMD_START received — checking ToF sensors...");

    /* ------------------------------------------------------------------ */
    /* Pre-arm safety: require a complete frame from every ToF sensor.      */
    /* Configuration/start success alone does not prove obstacle coverage. */
    /* ------------------------------------------------------------------ */
    {
        int tof_ok = tof_sensors_ok_count();
        while (tof_ok < TOF_SENSOR_COUNT) {
            ESP_LOGE(TAG, "TOF CHECK FAILED: %d / %d sensors frame-ready "
                     "— refusing to arm",
                     tof_ok, TOF_SENSOR_COUNT);
            vTaskDelay(pdMS_TO_TICKS(2000));
            tof_ok = tof_sensors_ok_count();
        }
        ESP_LOGI(TAG, "All %d ToF sensors frame-ready — checking freshness",
                 tof_ok);
    }

    /* Initialisation alone is not enough: wait until every channel has
     * published a complete, recent frame before changing flight mode. */
    while (!tof_all_sensors_fresh(TOF_NAV_MAX_FRAME_AGE_MS)) {
        const tof_health_t health = tof_get_health(TOF_NAV_MAX_FRAME_AGE_MS);
        ESP_LOGW(TAG,
                 "Waiting for full ToF coverage: online=0x%02x fresh=0x%02x",
                 (unsigned)health.online_mask,
                 (unsigned)health.fresh_mask);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "All ToF channels have fresh frames — flight gate open");

    /* ------------------------------------------------------------------ */
    /* Phase 3a: Switch to OFFBOARD mode                                   */
    /* Retry every 500 ms until PX4 confirms via heartbeat custom_main_mode */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Requesting OFFBOARD mode...");
    while (1) {
        if (at_detect_land_requested()) goto precision_landing;
        mavlink_set_offboard_mode();
        vTaskDelay(pdMS_TO_TICKS(500));
        if (mavlink_get_state().custom_main_mode == PX4_MAIN_MODE_OFFBOARD) {
            ESP_LOGI(TAG, "OFFBOARD mode confirmed");
            break;
        }
        ESP_LOGW(TAG, "OFFBOARD not set yet — retrying");
    }

    /* ------------------------------------------------------------------ */
    /* Phase 3b: Arm                                                        */
    /* Retry every 500 ms until PX4 confirms armed via heartbeat           */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Arming...");
    while (1) {
        // if (at_detect_land_requested()) goto precision_landing;
        mavlink_arm(true);
        vTaskDelay(pdMS_TO_TICKS(500));
        if (mavlink_get_state().armed) {
            ESP_LOGI(TAG, "Armed confirmed");
            break;
        }
        ESP_LOGW(TAG, "Not armed yet — retrying");
    }

    /* ------------------------------------------------------------------ */
    /* Phase 4: Take off to cruise altitude                                */
    /* nav_task is still IDLE — mission_task owns the setpoint.            */
    /* ------------------------------------------------------------------ */
    {
        drone_state_t st = mavlink_get_state();
        takeoff_x = st.x;
        takeoff_y = st.y;
        takeoff_heading = st.heading;
        mavlink_set_position_ned(takeoff_x, takeoff_y, target_z,
                                 takeoff_heading);
    }
    ESP_LOGI(TAG, "Taking off to %.1f m AGL (NED z=%.2f)...", CRUISE_ALT_M, target_z);

    bool altitude_reached = false;
    for (int healthy_ticks = 0; healthy_ticks < TAKEOFF_TIMEOUT_S * 10; ) {
        if (!tof_all_sensors_fresh(TOF_NAV_MAX_FRAME_AGE_MS)) {
            /* nav_task has latched a hold. Pause the takeoff timeout and do
             * not overwrite that hold until full coverage returns. */
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Reassert the mission-owned takeoff target after a recovered ToF
         * hold; mavlink_task will continue streaming the newest setpoint. */
        mavlink_set_position_ned(takeoff_x, takeoff_y, target_z,
                                 takeoff_heading);
        if (fabsf(mavlink_get_state().z - target_z) < ALT_TOLERANCE_M) {
            altitude_reached = true;
            break;
        }
        ++healthy_ticks;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (altitude_reached) {
        ESP_LOGI(TAG, "Altitude reached: NED z=%.2f (target=%.2f)",
                 mavlink_get_state().z, target_z);
    } else {
        ESP_LOGE(TAG,
                 "Takeoff altitude timeout: NED z=%.2f (target=%.2f) — holding",
                 mavlink_get_state().z, target_z);
        mavlink_set_hold();
        nav_goal_active = false;
        goto do_land;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* ------------------------------------------------------------------ */
    /* Phase 5: Laptop-driven exploration                                 */
    /* Goals arrive via CMD_GOTO from the laptop (handled in wifi_task).  */
    /* This loop just monitors for landing triggers.                       */
    /* ------------------------------------------------------------------ */
explore_loop:
    nav_goal_active = true;   /* wifi_task owns nav goals from here */
    ESP_LOGI(TAG, "Exploration mode — waiting for laptop goals...");

    while (1) {
        if (at_detect_land_requested()) goto precision_landing;
        if (wifi_land_requested()) {
            wifi_clear_land_request();
            ESP_LOGI(TAG, "CMD_LAND received from laptop");
            nav_goal_active = false;
            goto do_land;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* ------------------------------------------------------------------ */
    /* Phase 6a: Fly to tag position, refreshing pose on each detection   */
    /* Times out after 60 s and returns to exploration.                    */
    /* ------------------------------------------------------------------ */
precision_landing:
    ESP_LOGI(TAG, "Precision landing: tag %d", at_detect_last_id());

    if (nav_goal_active || nav_get_status().state != NAV_IDLE) {
        nav_cancel();
        nav_goal_active = false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    {
#define PRECISION_TIMEOUT_MS  30000
#define POSE_UPDATE_THRESH_M  0.15f

        uint32_t pl_start_ms  = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t last_detect_ms = 0;
        float    goal_x = mavlink_get_state().x;
        float    goal_y = mavlink_get_state().y;

        /* Seed goal from first available pose */
        {
            at_detect_pose_t p0 = at_detect_get_pose();
            if (p0.valid) {
                float dn, de, hdist;
                camera_to_ned(p0.tx, p0.ty, p0.tz, p0.drone_heading,
                              &dn, &de, &hdist);
                float odom_x = p0.drone_x + dn;
                float odom_y = p0.drone_y + de;
                odom_to_map(odom_x, odom_y, &goal_x, &goal_y);
                last_detect_ms = p0.detect_ms;
                ESP_LOGI(TAG, "Initial tag map (%.2f, %.2f), hdist=%.2f m",
                         goal_x, goal_y, hdist);
            } else {
                ESP_LOGW(TAG, "No valid pose yet — will refine when tag seen");
                odom_to_map(goal_x, goal_y, &goal_x, &goal_y);
            }
        }
        nav_set_goal_map(goal_x, goal_y, target_z);

        for (;;) {
            /* Check timeout */
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if ((now_ms - pl_start_ms) >= PRECISION_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Precision landing timed out — returning to exploration");
                nav_cancel();
                at_detect_reset_latch();
                goto explore_loop;
            }

            /* Check for a fresh pose reading — but stop updating once
             * we're close to the goal.  Pose updates call nav_set_goal_map
             * which resets nav state to NAV_ROTATING, preventing arrival. */
            nav_status_t ns = nav_get_status();
            bool close_enough = ns.dist_to_goal < NAV_ARRIVE_RADIUS_M * 2.0f;

            at_detect_pose_t pose = at_detect_get_pose();
            if (!close_enough && pose.valid && pose.detect_ms != last_detect_ms) {
                last_detect_ms = pose.detect_ms;
                float dn, de, hdist;
                camera_to_ned(pose.tx, pose.ty, pose.tz, pose.drone_heading,
                              &dn, &de, &hdist);
                float odom_x = pose.drone_x + dn;
                float odom_y = pose.drone_y + de;
                float map_x, map_y;
                odom_to_map(odom_x, odom_y, &map_x, &map_y);
                float dx = map_x - goal_x;
                float dy = map_y - goal_y;
                if (sqrtf(dx*dx + dy*dy) > POSE_UPDATE_THRESH_M) {
                    goal_x = map_x;
                    goal_y = map_y;
                    nav_set_goal_map(goal_x, goal_y, target_z);
                    ESP_LOGI(TAG, "Pose updated → map (%.2f, %.2f), hdist=%.2f m",
                             goal_x, goal_y, hdist);
                }
            }

            if (ns.state == NAV_ARRIVED) {
                ESP_LOGI(TAG, "Above tag — landing");
                nav_cancel();
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    /* ------------------------------------------------------------------ */
    /* Phase 6b: Land                                                       */
    /* ------------------------------------------------------------------ */
do_land:
    mavlink_send_land_command();
    ESP_LOGI(TAG, "Land command sent (tag %d)", at_detect_last_id());

    for (int i = 0; i < LAND_TIMEOUT_S * 10; i++) {
        if (!mavlink_get_state().armed) {
            ESP_LOGI(TAG, "Disarmed — mission complete");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}
#endif

/* ---------------------------------------------------------------------------
 * app_main — init and spawn all tasks
 * --------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP starting");

    /* NVS must be init before WiFi */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* Init all modules before spawning tasks */
    experiment_init();
    mavlink_task_init();
    tof_task_init();
#if CONFIG_NANOSLAM_ENABLED
    slam_bridge_init();   /* after tof_task_init: reads the sensor angle table */
#endif
    nav_task_init();
    at_detect_init();
    odom_init();
    wifi_task_init();   /* non-blocking; reconnect continues in background */

    /* Core 0: hardware-facing tasks + WiFi telemetry */
    xTaskCreatePinnedToCore(
        mavlink_task, "mav", MAV_TASK_STACK,
        NULL, MAV_TASK_PRIORITY, NULL, MAV_TASK_CORE
    );
#if !CONFIG_EXPERIMENT_WITHOUT_TOF
    xTaskCreatePinnedToCore(
        tof_task, "tof", TOF_TASK_STACK,
        NULL, TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE
    );
#else
    ESP_LOGW(TAG, "BASELINE FLIGHT: ToF acquisition disabled; no ToF obstacle checks");
#endif
    xTaskCreatePinnedToCore(
        wifi_task, "wifi", WIFI_TASK_STACK,
        NULL, WIFI_TASK_PRIORITY, NULL, WIFI_TASK_CORE
    );
#if CONFIG_NANOSLAM_ENABLED
    xTaskCreatePinnedToCore(
        slam_task, "slam", SLAM_TASK_STACK,
        NULL, SLAM_TASK_PRIORITY, NULL, SLAM_TASK_CORE
    );
#endif

#if CONFIG_APRILTAG_CAMERA_ENABLED
    /* Core 1 is dedicated to the optional camera/AprilTag pipeline. */
    xTaskCreatePinnedToCore(
        at_detect_task, "apriltag", AT_TASK_STACK,
        NULL, AT_TASK_PRIORITY, NULL, AT_TASK_CORE
    );
#else
    ESP_LOGW(TAG,
             "AprilTag camera disabled; camera hardware will not be initialized");
#endif

#if CONFIG_BENCH_MODE
    /* Development default: observe the real sensor/estimator workload without
     * starting tasks that can issue navigation or mission flight commands. */
    ESP_LOGW(TAG,
             "BENCH MODE: navigation and mission tasks are disabled");
#if !CONFIG_APRILTAG_CAMERA_ENABLED
    ESP_LOGI(TAG,
             "Core 0 runs MAVLink/ToF/Wi-Fi/NanoSLAM; Core 1 is reserved");
#endif
#elif CONFIG_EXPERIMENT_MISSION_ENABLED
    xTaskCreatePinnedToCore(experiment_task, "experiment", 6144, NULL, 3, NULL, 1);
#else
    /* Core 1: navigator (Pri 3) + mission (Pri 2)
     * nav_task preempts mission_task on Core 1 when it has work. */
    xTaskCreatePinnedToCore(
        nav_task, "nav", NAV_TASK_STACK,
        NULL, NAV_TASK_PRIORITY, NULL, NAV_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        mission_task, "mission", 6144,
        NULL, 2, NULL, 1
    );
#endif

    ESP_LOGI(TAG, "All tasks spawned");
}
