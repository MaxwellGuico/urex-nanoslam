#include "tof_task.h"
#include "mavlink_task.h"

#include "vl53l5cx_api.h"
#include "platform.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>
#include <float.h>

static const char *TAG = "tof";
_Static_assert(VL53L5CX_NB_TARGET_PER_ZONE==1, "Ring acquisition requires one target per zone");

// ---------------------------------------------------------------------------
// Sensor angle table (body frame, degrees, CW from forward)
// Computed in tof_task_init() from TOF_FRONT_SENSOR_IDX.
// ---------------------------------------------------------------------------
static float SENSOR_ANGLES[TOF_SENSOR_COUNT];

// ---------------------------------------------------------------------------
// Shared scan state — written by tof_task, read by mission_task
// ---------------------------------------------------------------------------
static tof_scan_t    s_scan;
static SemaphoreHandle_t s_scan_mutex;
static QueueHandle_t s_records;
static uint32_t s_record_drops, s_sequences[TOF_SENSOR_COUNT];
static uint64_t s_not_ready[TOF_SENSOR_COUNT];

// ---------------------------------------------------------------------------
// I2C / hardware handles (private to this file)
// ---------------------------------------------------------------------------
static i2c_master_bus_handle_t  s_bus_handle;
static i2c_master_dev_handle_t  s_tca_handle;
static SemaphoreHandle_t        s_bus_mutex;
static QueueHandle_t            s_recovery_queue;

// Each mux channel needs independent ST driver state. In addition to the
// stream counter, initialization and frame parsing use per-device buffers and
// configuration pointers that must not be overwritten by another sensor.
static VL53L5CX_Configuration   s_dev[TOF_SENSOR_COUNT];
// Lifecycle is deliberately split: configured/ranging controls whether the
// acquisition task may poll a channel, while sensor_ok is published only
// after at least one complete frame has actually arrived.
static uint8_t                  s_sensor_configured[TOF_SENSOR_COUNT];
static uint8_t                  s_sensor_ranging[TOF_SENSOR_COUNT];
static uint8_t                  s_sensor_ok[TOF_SENSOR_COUNT];
static uint8_t                  s_recovery_pending[TOF_SENSOR_COUNT];
static uint32_t                 s_last_frame_ms[TOF_SENSOR_COUNT];
static uint32_t                 s_last_reinit_ms[TOF_SENSOR_COUNT];
static bool                     s_ring_frame_ready_reported;

#define TOF_MUX_I2C_SPEED_HZ        400000
#define TOF_BUS_FAIL_THRESHOLD      10
#define TOF_SENSOR_STALE_MS         3000U
#define TOF_SENSOR_REINIT_MS        2000U
#define TOF_RECOVERY_TASK_STACK     6144U
#define TOF_RECOVERY_TASK_PRIORITY  (TOF_TASK_PRIORITY - 1)

#ifndef CONFIG_VL53L5CX_MUXED_BUS
#error "The ToF ring requires CONFIG_VL53L5CX_MUXED_BUS"
#endif

static void publish_sensor_lifecycle(uint8_t sensor, bool configured,
                                     bool ranging, uint32_t now_ms)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    s_sensor_configured[sensor] = configured ? 1U : 0U;
    s_sensor_ranging[sensor] = ranging ? 1U : 0U;
    s_sensor_ok[sensor] = 0U;
    s_scan.sensor_ok[sensor] = 0U;
    s_scan.frame[sensor].valid = 0U;
    s_not_ready[sensor] = 0;
    s_last_frame_ms[sensor] = ranging ? now_ms : 0U;
    s_ring_frame_ready_reported = false;
    xSemaphoreGive(s_scan_mutex);
}

static bool sensor_is_pollable(uint8_t sensor)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    const bool pollable = s_sensor_ranging[sensor] != 0U;
    xSemaphoreGive(s_scan_mutex);
    return pollable;
}

// ---------------------------------------------------------------------------
// Compute minimum range (metres) from a stored tof_frame_t.
//
// Status 255: no target detected — ignored.
// Any other status: use the reported distance. Status 5/9 are fully valid;
// lower-confidence returns (e.g. status 10, "no target at previous range",
// which appears in most frames) still count if they report something close,
// so the obstacle check stays conservative without inventing a range.
// Previously non-5/9 zones were forced to 0.30 m, which made every status-10
// zone abort the mission even when it measured 2 m. Sensor loss is covered
// separately by the freshness check.
// Returns INFINITY only if every pixel is status 255 or out-of-range.
// ---------------------------------------------------------------------------
static float frame_min_range_m(const tof_frame_t *f)
{
    float min_m = INFINITY;
    for (int row = TOF_NATIVE_SIDE / 2; row < TOF_NATIVE_SIDE; row++) {
        for (int col = 0; col < TOF_NATIVE_SIDE; col++) {
            int      i      = row * TOF_NATIVE_SIDE + col;
            uint8_t  status = f->target_status[i];
            int16_t  dist   = f->distance_mm[i];

            if (status == 255) continue;
            if (dist < TOF_MIN_VALID_MM || dist > TOF_MAX_VALID_MM) continue;
            float m = (float)dist / 1000.0f;
            if (m < min_m) min_m = m;
        }
    }
    return min_m;
}

// ---------------------------------------------------------------------------
// Configure one sensor via the mux without starting continuous ranging.
// Returns true on success.
// ---------------------------------------------------------------------------
static bool configure_sensor(uint8_t idx)
{
    VL53L5CX_Configuration *dev = &s_dev[idx];

    uint8_t alive = 0;
    if (vl53l5cx_is_alive(dev, &alive) || !alive) {
        ESP_LOGW(TAG, "[%d] not detected", idx);
        return false;
    }

    if (vl53l5cx_init(dev)) {
        ESP_LOGE(TAG, "[%d] init failed", idx);
        return false;
    }

    if (vl53l5cx_set_resolution(dev, TOF_NATIVE_PIXELS) ||
        vl53l5cx_set_ranging_mode(dev, VL53L5CX_RANGING_MODE_AUTONOMOUS) ||
        vl53l5cx_set_integration_time_ms(dev, TOF_INTEGRATION_MS) ||
        vl53l5cx_set_ranging_frequency_hz(dev, TOF_RANGING_FREQ_HZ)) {
        ESP_LOGE(TAG, "[%d] profile configuration failed", idx);
        return false;
    }
    uint8_t resolution=0, frequency=0, mode=0;
    uint32_t integration=0;
    if (vl53l5cx_get_resolution(dev,&resolution) ||
        vl53l5cx_get_ranging_frequency_hz(dev,&frequency) ||
        vl53l5cx_get_ranging_mode(dev,&mode) ||
        vl53l5cx_get_integration_time_ms(dev,&integration) ||
        resolution!=TOF_NATIVE_PIXELS || frequency!=TOF_RANGING_FREQ_HZ ||
        mode!=VL53L5CX_RANGING_MODE_AUTONOMOUS || integration!=TOF_INTEGRATION_MS) {
        ESP_LOGE(TAG,"[%d] profile readback mismatch",idx);
        return false;
    }
    ESP_LOGI(TAG, "[%d] configured — %.0f° @ %dHz %dx%d, autonomous, integration=%lums",
             idx, SENSOR_ANGLES[idx], TOF_RANGING_FREQ_HZ, TOF_NATIVE_SIDE, TOF_NATIVE_SIDE,
             (unsigned long)TOF_INTEGRATION_MS);
    return true;
}

static bool start_sensor_ranging(uint8_t idx)
{
    if (vl53l5cx_start_ranging(&s_dev[idx])) {
        ESP_LOGE(TAG, "[%u] start ranging failed", (unsigned)idx);
        return false;
    }
    ESP_LOGI(TAG, "[%u] ranging started — awaiting first frame",
             (unsigned)idx);
    return true;
}

static bool reinitialize_and_start_sensor(uint8_t idx)
{
    return configure_sensor(idx) && start_sensor_ranging(idx);
}

static bool schedule_sensor_recovery(uint8_t sensor, uint32_t now_ms,
                                     bool force)
{
    bool should_queue = false;

    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    if (!s_sensor_ranging[sensor] && !s_recovery_pending[sensor] &&
        (force ||
         (uint32_t)(now_ms - s_last_reinit_ms[sensor]) >=
             TOF_SENSOR_REINIT_MS)) {
        s_recovery_pending[sensor] = 1U;
        s_last_reinit_ms[sensor] = now_ms;
        should_queue = true;
    }
    xSemaphoreGive(s_scan_mutex);

    if (!should_queue) {
        return false;
    }

    if (xQueueSend(s_recovery_queue, &sensor, 0) != pdTRUE) {
        xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
        s_recovery_pending[sensor] = 0U;
        xSemaphoreGive(s_scan_mutex);
        ESP_LOGE(TAG, "[%u] recovery queue full", (unsigned)sensor);
        return false;
    }

    ESP_LOGW(TAG, "[%u] recovery queued; healthy polling continues",
             (unsigned)sensor);
    return true;
}

static bool mark_sensor_stale(uint8_t sensor, uint32_t now_ms,
                              uint32_t *age_ms)
{
    bool stale = false;

    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    const uint32_t age = now_ms - s_last_frame_ms[sensor];
    if (s_sensor_ranging[sensor] && age >= TOF_SENSOR_STALE_MS) {
        s_sensor_configured[sensor] = 0U;
        s_sensor_ranging[sensor] = 0U;
        s_sensor_ok[sensor] = 0U;
        s_scan.sensor_ok[sensor] = 0U;
        s_scan.frame[sensor].valid = 0U;
        s_ring_frame_ready_reported = false;
        stale = true;
    }
    xSemaphoreGive(s_scan_mutex);

    if (age_ms != NULL) {
        *age_ms = age;
    }
    return stale;
}

static void tof_recovery_task(void *arg)
{
    (void)arg;
    uint8_t sensor = 0U;

    for (;;) {
        if (xQueueReceive(s_recovery_queue, &sensor, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        const uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
        ESP_LOGW(TAG, "[%u] background full recovery started",
                 (unsigned)sensor);
        const bool recovered = reinitialize_and_start_sensor(sensor);
        const uint32_t end_ms = (uint32_t)(esp_timer_get_time() / 1000);

        xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
        s_sensor_configured[sensor] = recovered ? 1U : 0U;
        s_sensor_ranging[sensor] = recovered ? 1U : 0U;
        s_sensor_ok[sensor] = 0U;
        s_scan.sensor_ok[sensor] = 0U;
        s_scan.frame[sensor].valid = 0U;
        s_not_ready[sensor]=0;
        s_last_reinit_ms[sensor] = end_ms;
        s_last_frame_ms[sensor] = recovered ? end_ms : 0U;
        s_ring_frame_ready_reported = false;
        s_recovery_pending[sensor] = 0U;
        xSemaphoreGive(s_scan_mutex);

        if (recovered) {
            ESP_LOGI(TAG,
                     "[%u] background recovery restarted ranging in %lu ms; "
                     "awaiting first fresh frame",
                     (unsigned)sensor,
                     (unsigned long)(end_ms - start_ms));
        } else {
            ESP_LOGW(TAG,
                     "[%u] background recovery failed after %lu ms; "
                     "retry remains scheduled",
                     (unsigned)sensor,
                     (unsigned long)(end_ms - start_ms));
        }
    }
}

// ---------------------------------------------------------------------------
// Angle helpers
// ---------------------------------------------------------------------------

// Normalise angle to [0, 360)
static float norm360(float a)
{
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

// Shortest angular distance (unsigned) between two angles, result in [0, 180]
static float angle_dist(float a, float b)
{
    float d = fabsf(norm360(a) - norm360(b));
    if (d > 180.0f) d = 360.0f - d;
    return d;
}

// True if sensor at 'sensor_center_deg' has any coverage overlap with
// the query arc [q_min_deg, q_max_deg].
//
// Logic: the sensor covers ±HALF_WIDTH around its center.
// The query covers from q_min to q_max (may cross 0°).
// Overlap exists if the closest point on the query arc to the sensor center
// is within HALF_WIDTH degrees.
static bool sensor_overlaps_sector(float sensor_center_deg,
                                   float q_min_deg, float q_max_deg)
{
    float sc = norm360(sensor_center_deg);
    float qn = norm360(q_min_deg);
    float qx = norm360(q_max_deg);

    // Check if the sensor center falls inside the query arc
    bool inside;
    if (qn <= qx) {
        inside = (sc >= qn && sc <= qx);
    } else {
        // arc crosses 0°
        inside = (sc >= qn || sc <= qx);
    }
    if (inside) return true;

    // Otherwise, check if either query endpoint is within HALF_WIDTH of sensor center
    return angle_dist(sc, qn) <= TOF_SENSOR_HALF_WIDTH_DEG ||
           angle_dist(sc, qx) <= TOF_SENSOR_HALF_WIDTH_DEG;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void tof_task_init(void)
{
    memset(&s_scan, 0, sizeof(s_scan));
    memset(s_dev, 0, sizeof(s_dev));
    memset(s_sensor_configured, 0, sizeof(s_sensor_configured));
    memset(s_sensor_ranging, 0, sizeof(s_sensor_ranging));
    memset(s_sensor_ok, 0, sizeof(s_sensor_ok));
    memset(s_recovery_pending, 0, sizeof(s_recovery_pending));
    memset(s_last_frame_ms, 0, sizeof(s_last_frame_ms));
    memset(s_last_reinit_ms, 0, sizeof(s_last_reinit_ms));
    s_ring_frame_ready_reported = false;

    // Compute CCW sensor angles from front sensor index
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        SENSOR_ANGLES[i] = (float)(((TOF_FRONT_SENSOR_IDX - i) * 45 % 360 + 360) % 360);
    }

    s_records = xQueueCreate(64, sizeof(tof_record_t));
    configASSERT(s_records != NULL);
    s_scan_mutex = xSemaphoreCreateMutex();
    configASSERT(s_scan_mutex != NULL);
    s_bus_mutex = xSemaphoreCreateMutex();
    configASSERT(s_bus_mutex != NULL);
    s_recovery_queue = xQueueCreate(TOF_SENSOR_COUNT, sizeof(uint8_t));
    configASSERT(s_recovery_queue != NULL);
}

float tof_get_min_range_in_sector(float angle_min_deg, float angle_max_deg)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    tof_scan_t snap = s_scan;
    xSemaphoreGive(s_scan_mutex);

    float min_m = INFINITY;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (!snap.sensor_ok[i])  continue;
        if (!snap.frame[i].valid) continue;
        if (!sensor_overlaps_sector(SENSOR_ANGLES[i], angle_min_deg, angle_max_deg)) continue;

        float m = frame_min_range_m(&snap.frame[i]);
        if (m < min_m) min_m = m;
    }
    return min_m;
}

bool tof_pop_record(tof_record_t *record)
{ return xQueueReceive(s_records, record, 0) == pdTRUE; }
uint32_t tof_record_drops(void)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    uint32_t drops=s_record_drops;
    xSemaphoreGive(s_scan_mutex);
    return drops;
}

tof_scan_t tof_get_scan(void)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    tof_scan_t copy = s_scan;
    xSemaphoreGive(s_scan_mutex);
    return copy;
}

float tof_sensor_angle_deg(uint8_t sensor_id)
{
    return sensor_id < TOF_SENSOR_COUNT ? SENSOR_ANGLES[sensor_id] : NAN;
}

tof_health_t tof_get_health(uint32_t max_frame_age_ms)
{
    tof_health_t health;
    memset(&health, 0, sizeof(health));
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        health.age_ms[i] = UINT32_MAX;
    }

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        const uint8_t bit = (uint8_t)(1U << i);
        if (s_scan.sensor_ok[i]) {
            health.online_mask |= bit;
        }
        if (s_scan.sensor_ok[i] && s_scan.frame[i].valid) {
            const uint32_t age_ms = now_ms - s_scan.frame[i].timestamp_ms;
            health.age_ms[i] = age_ms;
            if (age_ms <= max_frame_age_ms) {
                health.fresh_mask |= bit;
            }
        }
    }
    xSemaphoreGive(s_scan_mutex);

    return health;
}

bool tof_all_sensors_fresh(uint32_t max_frame_age_ms)
{
    const tof_health_t health = tof_get_health(max_frame_age_ms);
    return health.online_mask == TOF_ALL_SENSOR_MASK &&
           health.fresh_mask == TOF_ALL_SENSOR_MASK;
}

bool tof_sector_is_fresh(float angle_min_deg, float angle_max_deg,
                         uint32_t max_frame_age_ms,
                         uint8_t *required_mask, uint8_t *missing_mask)
{
    uint8_t required = 0U;
    for (uint8_t sensor = 0U; sensor < TOF_SENSOR_COUNT; ++sensor) {
        if (sensor_overlaps_sector(SENSOR_ANGLES[sensor], angle_min_deg,
                                   angle_max_deg)) {
            required |= (uint8_t)(1U << sensor);
        }
    }

    const tof_health_t health = tof_get_health(max_frame_age_ms);
    const uint8_t missing = (uint8_t)(required & ~health.fresh_mask);
    if (required_mask != NULL) {
        *required_mask = required;
    }
    if (missing_mask != NULL) {
        *missing_mask = missing;
    }

    return required != 0U && missing == 0U;
}

int tof_sensors_ok_count(void)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    int count = 0;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++)
        count += s_scan.sensor_ok[i];
    xSemaphoreGive(s_scan_mutex);
    return count;
}

tof_scan_collapsed_t tof_get_collapsed_scan(void)
{
    tof_scan_t snap = tof_get_scan();  /* thread-safe snapshot */

    tof_scan_collapsed_t out;
    for (int i = 0; i < TOF_SENSOR_COUNT * TOF_SENSOR_RESO; i++)
        out.ranges[i] = INFINITY;

    for (int s = 0; s < TOF_SENSOR_COUNT; s++) {
        if (!snap.sensor_ok[s] || !snap.frame[s].valid) continue;
        const tof_frame_t *f = &snap.frame[s];

        for (int col = 0; col < TOF_NATIVE_SIDE; col++) {
            /* Sensors right-side up: raw col 0 = RIGHT of FoV (from lens).
             * col 0 → +offset (CW), col 7 → −offset (CCW). */
            float angle_deg = SENSOR_ANGLES[s] + ((TOF_NATIVE_SIDE - 1) * 0.5f - (float)col) * (45.0f / TOF_NATIVE_SIDE);

            /* Rows 4-7 = physical upper half (sensors mounted upside-down) */
            for (int row = TOF_NATIVE_SIDE / 2; row < TOF_NATIVE_SIDE; row++) {
                int      px     = row * TOF_NATIVE_SIDE + col;
                uint8_t  status = f->target_status[px];
                int16_t  dist   = f->distance_mm[px];

                /* Status 255: out of range (nothing there) — leave INFINITY.
                 * Status 5/9: valid measurement — use reported distance.
                 * Anything else: unreliable — assume 0.3 m (conservative). */
                float dist_m;
                if (status == 255) {
                    continue;   /* open space, INFINITY is correct */
                } else if (status == 5 || status == 9) {
                    if (dist < TOF_MIN_VALID_MM || dist > TOF_MAX_VALID_MM) continue;
                    dist_m = (float)dist / 1000.0f;
                } else {
                    dist_m = 0.40f;  /* conservative fallback for error statuses */
                }

                float norm_angle = fmodf(angle_deg, 360.0f);
                if (norm_angle < 0.0f) norm_angle += 360.0f;
                int idx = (int)(norm_angle * (float)(TOF_SENSOR_COUNT * TOF_SENSOR_RESO) / 360.0f);
                if (idx >= TOF_SENSOR_COUNT * TOF_SENSOR_RESO) idx = TOF_SENSOR_COUNT * TOF_SENSOR_RESO - 1;

                if (dist_m < out.ranges[idx])
                    out.ranges[idx] = dist_m;
                // printf("(%d: %zu)", idx, dist);
            }
            // printf("\n");
        }
    }

    return out;
}

// ---------------------------------------------------------------------------
// FreeRTOS task
// ---------------------------------------------------------------------------

void tof_task(void *arg)
{
    // ---- I2C master bus ----
    i2c_master_bus_config_t bus_cfg = {
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .i2c_port               = TOF_I2C_PORT,
        .scl_io_num             = TOF_SCL_PIN,
        .sda_io_num             = TOF_SDA_PIN,
        .glitch_ignore_cnt      = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus_handle));
    ESP_LOGI(TAG, "I2C master bus created");

    // ---- TCA9548A mux ----
    i2c_device_config_t tca_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TCA_I2C_ADDR,
        .scl_speed_hz    = TOF_MUX_I2C_SPEED_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus_handle, &tca_cfg, &s_tca_handle));
    ESP_LOGI(TAG, "TCA9548A registered");

    // ---- VL53L5CX transport shared by independent per-channel drivers ----
    i2c_device_config_t vl_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1,
        .scl_speed_hz    = TOF_MUX_I2C_SPEED_HZ,
    };
    i2c_master_dev_handle_t vl_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(
        s_bus_handle, &vl_cfg, &vl_handle));
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        s_dev[i].platform.bus_config = bus_cfg;
        s_dev[i].platform.handle = vl_handle;
        s_dev[i].platform.sensor_id = (uint8_t)i;
#ifdef CONFIG_VL53L5CX_MUXED_BUS
        s_dev[i].platform.mux_handle = s_tca_handle;
        s_dev[i].platform.bus_mutex = s_bus_mutex;
        s_dev[i].platform.mux_channel = (uint8_t)i;
#endif
    }

    // ---- Configure all 8 sensors without starting continuous ranging ----
    // Keeping every device idle during this phase prevents early channels
    // from ranging unpolled while a later channel is retried.
    for (;;) {
        int configured_count = 0;
        for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
            if (s_sensor_configured[i]) {
                configured_count++;
                continue;
            }

            const bool configured = configure_sensor((uint8_t)i);
            publish_sensor_lifecycle((uint8_t)i, configured, false, 0U);
            if (!configured) {
                // Bus may be stuck — reset and give it a moment before next pass
                VL53L5CX_InvalidateMuxCache();
                const esp_err_t err = i2c_master_bus_reset(s_bus_handle);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG,
                             "startup bus reset esp_err=0x%08lx "
                             "esp_err_name=%s",
                             (unsigned long)(uint32_t)err,
                             esp_err_to_name(err));
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (configured) {
                configured_count++;
            }
        }

        ESP_LOGI(TAG, "%d / %d sensors configured (ranging not started)",
                 configured_count, TOF_SENSOR_COUNT);
        if (configured_count == TOF_SENSOR_COUNT) {
            break;
        }

        ESP_LOGW(TAG, "Retrying failed sensor configuration in 500 ms...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // ---- Start every configured channel once, in a bounded sequence ----
    // No channel is reported online yet. A channel becomes online only after
    // the polling loop receives its first complete frame.
    uint8_t ranging_mask = 0U;
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        const bool ranging = start_sensor_ranging((uint8_t)i);
        const uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
        publish_sensor_lifecycle((uint8_t)i, true, ranging, start_ms);
        if (ranging) {
            ranging_mask |= (uint8_t)(1U << i);
        }
    }
    ESP_LOGI(TAG, "ToF transfer=%lu B/sensor frame; target=%d Hz; payload load >=%lu B/s on 400kHz bus",
             (unsigned long)s_dev[0].data_read_size, TOF_RANGING_FREQ_HZ,
             (unsigned long)(s_dev[0].data_read_size*TOF_SENSOR_COUNT*TOF_RANGING_FREQ_HZ));
    const uint32_t wire_min_bps=s_dev[0].data_read_size*TOF_SENSOR_COUNT*TOF_RANGING_FREQ_HZ*9;
    if (wire_min_bps>=TOF_MUX_I2C_SPEED_HZ)
        ESP_LOGW(TAG,"Requested aggregate rate exceeds 400kHz even before mux/poll overhead: %lu bit/s minimum",
                 (unsigned long)wire_min_bps);
    ESP_LOGI(TAG,
             "start sequence complete: configured_mask=0x%02x "
             "ranging_mask=0x%02x frame_ready_mask=0x00",
             TOF_ALL_SENSOR_MASK, ranging_mask);

    const BaseType_t recovery_task_created = xTaskCreatePinnedToCore(
        tof_recovery_task,
        "tof_recover",
        TOF_RECOVERY_TASK_STACK,
        NULL,
        TOF_RECOVERY_TASK_PRIORITY,
        NULL,
        TOF_TASK_CORE);
    configASSERT(recovery_task_created == pdPASS);
    ESP_LOGI(TAG,
             "background recovery ready: priority=%d chunk=%dB",
             TOF_RECOVERY_TASK_PRIORITY,
             CONFIG_VL53L5CX_I2C_WRITE_CHUNK_BYTES);

    // A failed start enters the same serialized full-recovery path as a
    // runtime dropout. Successfully started channels are polled immediately.
    const uint32_t recovery_queue_ms =
        (uint32_t)(esp_timer_get_time() / 1000);
    for (uint8_t sensor = 0U; sensor < TOF_SENSOR_COUNT; ++sensor) {
        if ((ranging_mask & (uint8_t)(1U << sensor)) == 0U) {
            schedule_sensor_recovery(sensor, recovery_queue_ms, true);
        }
    }

    // ---- Round-robin polling loop ----
    // Pattern from mapper.c: select mux, check data ready, extract if ready.
    // Recovery runs at lower priority and releases the bus between bounded
    // write chunks, so these polls continue while one channel is reinitialized.
    VL53L5CX_ResultsData results;

    int consecutive_fails = 0;

    for (uint8_t sensor = 0; ; sensor = (sensor + 1) % TOF_SENSOR_COUNT) {

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        // A sensor can remain responsive on I2C while never asserting data
        // ready. Treat that as an offline sensor instead of retaining its last
        // frame forever. Three seconds represents about 30 missed 8x8 frames,
        // which separates a stalled stream from ordinary polling jitter.
        uint32_t frame_age_ms = 0U;
        if (mark_sensor_stale(sensor, now_ms, &frame_age_ms)) {
            ESP_LOGW(TAG,
                     "[%u] no fresh frame for %lu ms — scheduling re-init",
                     (unsigned)sensor,
                     (unsigned long)frame_age_ms);
            schedule_sensor_recovery(sensor, now_ms, true);
        }

        // ---- I2C bus recovery ----
        if (consecutive_fails >= TOF_BUS_FAIL_THRESHOLD) {
            ESP_LOGE(TAG, "I2C bus hung (%d consecutive failures) — resetting",
                     consecutive_fails);

            // A genuine shared-bus failure affects every channel. Reset the
            // bus once, invalidate all frames, then let the background worker
            // recover channels while the polling task remains responsive.
            for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
                publish_sensor_lifecycle((uint8_t)i, false, false, 0U);
            }

            // Reset the I2C bus (sends 9 SCL clocks to release a stuck SDA)
            xSemaphoreTake(s_bus_mutex, portMAX_DELAY);
            VL53L5CX_InvalidateMuxCache();
            esp_err_t err = i2c_master_bus_reset(s_bus_handle);
            xSemaphoreGive(s_bus_mutex);
            if (err != ESP_OK) {
                ESP_LOGE(TAG,
                         "runtime bus reset esp_err=0x%08lx "
                         "esp_err_name=%s",
                         (unsigned long)(uint32_t)err,
                         esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(100));

            now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
                schedule_sensor_recovery((uint8_t)i, now_ms, true);
            }
            ESP_LOGW(TAG, "Bus reset complete — channel recoveries queued");

            consecutive_fails = 0;
            continue;
        }

        // Periodically re-attempt full configuration/start on any sensor that
        // is not currently range-pollable.
        if (!sensor_is_pollable(sensor)) {
            schedule_sensor_recovery(sensor, now_ms, false);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Sweep the small readiness headers rapidly and only spend the full
        // frame-transfer time on channels that have a result. The platform
        // layer atomically selects this sensor's mux channel for each transfer.
        uint8_t is_ready = 0;
        VL53L5CX_Configuration *dev = &s_dev[sensor];
        const uint64_t ready_before_us = esp_timer_get_time();
        uint8_t status = vl53l5cx_check_data_ready(dev, &is_ready);
        const uint64_t ready_after_us = esp_timer_get_time();
        if (status) {
            ESP_LOGW(TAG, "[%d] check_data_ready status=%d", sensor, status);
            consecutive_fails++;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (is_ready) {
            const uint8_t ready_streamcount = dev->streamcount;
            const uint64_t read_start_us = esp_timer_get_time();
            status = vl53l5cx_get_ranging_data(dev, &results);
            const uint64_t read_end_us = esp_timer_get_time();
            if (status) {
                ESP_LOGW(TAG, "[%d] get_ranging_data status=%d", sensor, status);
                consecutive_fails++;
            } else {
                now_ms = (uint32_t)(read_end_us / 1000);
                tof_record_t record = {
                    .sensor=sensor, .streamcount=dev->streamcount,
                    .changed_during_read=ready_streamcount!=dev->streamcount,
                    .sequence=++s_sequences[sensor],
                    .ready_before_us=ready_before_us, .ready_after_us=ready_after_us,
                    .read_start_us=read_start_us, .read_end_us=read_end_us,
                    /* If a new result arrived during readout, the previous poll
                     * cannot bound this result. Never claim that bound. */
                    .previous_not_ready_us=ready_streamcount==dev->streamcount ? s_not_ready[sensor] : 0,
                    .transfer_bytes=dev->data_read_size, .sync=mavlink_clock_status(),
                };
                memcpy(record.distance_mm, results.distance_mm, TOF_NATIVE_PIXELS*sizeof(int16_t));
                memcpy(record.target_status, results.target_status, TOF_NATIVE_PIXELS);
                if (xQueueSend(s_records,&record,0)!=pdTRUE) {
                    xSemaphoreTake(s_scan_mutex,portMAX_DELAY); ++s_record_drops; xSemaphoreGive(s_scan_mutex);
                }
                s_not_ready[sensor]=0;
                consecutive_fails = 0;
                bool first_frame = false;
                bool ring_frame_ready = false;
                uint8_t frame_ready_mask = 0U;

                // Copy the full 64-pixel frame into the shared scan buffer.
                // distance_mm and target_status are the same array sizes as
                // TOF_PIXELS_PER_SENSOR (VL53L5CX_RESOLUTION_8X8 * NB_TARGET_PER_ZONE).
                xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
                first_frame = s_sensor_ok[sensor] == 0U;
                s_last_frame_ms[sensor] = now_ms;
                memcpy(s_scan.frame[sensor].distance_mm,
                       results.distance_mm,
                       TOF_NATIVE_PIXELS * sizeof(int16_t));
                memcpy(s_scan.frame[sensor].target_status,
                       results.target_status,
                       TOF_NATIVE_PIXELS * sizeof(uint8_t));
                s_scan.frame[sensor].timestamp_ms = now_ms;
                s_scan.frame[sensor].valid        = 1;
                s_sensor_ok[sensor] = 1U;
                s_scan.sensor_ok[sensor] = 1U;
                for (uint8_t index = 0U; index < TOF_SENSOR_COUNT; ++index) {
                    if (s_sensor_ok[index]) {
                        frame_ready_mask |= (uint8_t)(1U << index);
                    }
                }
                if (frame_ready_mask == TOF_ALL_SENSOR_MASK &&
                    !s_ring_frame_ready_reported) {
                    s_ring_frame_ready_reported = true;
                    ring_frame_ready = true;
                }
                xSemaphoreGive(s_scan_mutex);

                if (first_frame) {
                    ESP_LOGI(TAG,
                             "[%u] first fresh frame received; "
                             "frame_ready_mask=0x%02x",
                             (unsigned)sensor, (unsigned)frame_ready_mask);
                }
                if (ring_frame_ready) {
                    ESP_LOGI(TAG,
                             "%d / %d sensors frame-ready: every channel "
                             "has supplied a fresh frame",
                             TOF_SENSOR_COUNT, TOF_SENSOR_COUNT);
                }
            }
        } else {
            s_not_ready[sensor] = ready_before_us;
            // The bus transaction succeeded; the per-sensor stale deadline
            // above handles a device that never becomes data-ready.
            consecutive_fails = 0;
        }

        // Yield once per full ring, avoiding an artificial millisecond per sensor.
        if (sensor == TOF_SENSOR_COUNT-1) vTaskDelay(pdMS_TO_TICKS(1));
    }
}
