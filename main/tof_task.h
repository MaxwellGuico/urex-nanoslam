#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "nanolite_tof.h"
#include "clock_sync.h"
#include "sdkconfig.h"

// ---------------------------------------------------------------------------
// Hardware config
// ---------------------------------------------------------------------------
#define TOF_SENSOR_COUNT        8
#define TOF_SENSOR_RESO         8
#define TOF_I2C_PORT            I2C_NUM_0
#define TOF_SDA_PIN             CONFIG_TCA_SDA_GPIO_PORT   // from menuconfig
#define TOF_SCL_PIN             CONFIG_TCA_SCL_GPIO_PORT
#define TCA_I2C_ADDR            0x70
#define TOF_RANGING_FREQ_HZ     CONFIG_TOF_RATE_HZ
#define TOF_INTEGRATION_MS      5U
#define TOF_NATIVE_SIDE         CONFIG_TOF_GRID_SIZE
#define TOF_NATIVE_PIXELS       (TOF_NATIVE_SIDE * TOF_NATIVE_SIDE)
#if (TOF_NATIVE_SIDE != 4 && TOF_NATIVE_SIDE != 8) || (TOF_NATIVE_SIDE == 8 && TOF_RANGING_FREQ_HZ > 15)
#error "Supported ToF profiles: 8x8 up to 15 Hz, 4x4 up to 60 Hz"
#endif
#define TOF_MAX_VALID_MM        3000    // discard readings above this (sensor max is 4m)
#define TOF_MIN_VALID_MM        50      // discard readings below this (sensor blind zone)
#define TOF_NAV_MAX_FRAME_AGE_MS 250U   // maximum frame age accepted by flight safety
#define TOF_ALL_SENSOR_MASK      ((uint8_t)((1U << TOF_SENSOR_COUNT) - 1U))

// FreeRTOS placement — Core 0, below mavlink but above mission
#define TOF_TASK_CORE           0
#define TOF_TASK_PRIORITY       4
#define TOF_TASK_STACK          8192

// ---------------------------------------------------------------------------
// Sensor angular layout
//
// Sensors are mounted COUNTER-CLOCKWISE around the ring.  The front-facing
// sensor index is set via menuconfig (TOF_FRONT_SENSOR_IDX).
//
// Angle for sensor i = ((front_idx - i) * 45) mod 360  (degrees, CW+)
//
// The angle table is computed at runtime in tof_task_init().
// ---------------------------------------------------------------------------
#define TOF_FRONT_SENSOR_IDX        CONFIG_TOF_FRONT_SENSOR_IDX
#define TOF_SENSOR_HALF_WIDTH_DEG   22.5f

// Number of pixels per sensor frame (8x8 grid × 1 target per zone).
// Matches VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE without
// pulling vl53l5cx_api.h into this header.
#define TOF_PIXELS_PER_SENSOR       TOF_SENSOR_RESO * TOF_SENSOR_RESO

// ---------------------------------------------------------------------------
// Per-sensor frame — raw data exactly as returned by vl53l5cx_get_ranging_data
// ---------------------------------------------------------------------------
typedef struct {
    int16_t  distance_mm[TOF_PIXELS_PER_SENSOR];  // raw signed driver values
    uint8_t  target_status[TOF_PIXELS_PER_SENSOR]; // 5 = valid, 9 = valid weak signal
    uint32_t timestamp_ms;                         // time this frame was received
    uint8_t  valid;                                // 1 once first frame has arrived
} tof_frame_t;

// ---------------------------------------------------------------------------
// Scan snapshot — full 8×8 frame for every sensor in the ring
// ---------------------------------------------------------------------------
typedef struct {
    tof_frame_t frame[TOF_SENSOR_COUNT];
    // 1 only after configuration, ranging start, and the first complete frame.
    uint8_t     sensor_ok[TOF_SENSOR_COUNT];
} tof_scan_t;

// Per-sensor communication health.  A frame is fresh only when the sensor is
// online, has published at least one complete frame, and that frame is no older
// than the caller's maximum age.  Target validity is deliberately separate:
// an empty scene can have a healthy fresh frame with no accepted target.
typedef struct {
    uint8_t  online_mask;
    uint8_t  fresh_mask;
    uint32_t age_ms[TOF_SENSOR_COUNT]; // UINT32_MAX until the first frame
} tof_health_t;

// ---------------------------------------------------------------------------
// Init & task entry
// ---------------------------------------------------------------------------

// Call once before spawning the task.
// Creates the I2C master bus internally.
void tof_task_init(void);

// FreeRTOS task entry — pin to Core 0, Priority 4
// xTaskCreatePinnedToCore(tof_task, "tof", TOF_TASK_STACK,
//                         NULL, TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE);
void tof_task(void *arg);

// ---------------------------------------------------------------------------
// Data access API (thread-safe, call from mission_task)
// ---------------------------------------------------------------------------

// Returns the minimum range (metres) across all sensors whose coverage arc
// overlaps [angle_min_deg, angle_max_deg].
//
// Angles are body-frame degrees, 0=forward, CW positive.
// The arc may cross 0° (e.g. angle_min=-30, angle_max=+30 for forward sector).
// Returns INFINITY if no valid sensor covers the requested arc.
//
// Primary use: Bug2 obstacle detection
//   Forward sector:    tof_get_min_range_in_sector(-30.0f, 30.0f)
//   Left boundary:     tof_get_min_range_in_sector(45.0f, 135.0f)
float tof_get_min_range_in_sector(float angle_min_deg, float angle_max_deg);

// Full scan snapshot — returns a copy of the latest scan data.
// Use for telemetry or when you need all readings at once.
tof_scan_t tof_get_scan(void);

/* Reduce the newest ring frames for mapping using zero-based rows 3 and 4.
 * Invalid/no-target returns remain invalid and therefore do not clear unknown
 * map space. Returns the number of valid reduced observations. */
uint8_t tof_get_mapping_returns(
    uint16_t map_effective_range_mm,
    nanolite_tof_return_t returns[TOF_SENSOR_COUNT]);

/* Body-frame sensor center bearing, degrees clockwise from the vehicle nose. */
float tof_sensor_angle_deg(uint8_t sensor_id);

// Collapse the 8-sensor ring into a flat 2D scan 
// For each sensor, get the minimum of each column for rows 3-7
// Maximum count = 64 (8 sensors × 8 columns).
// Thread-safe: takes the scan mutex internally via tof_get_scan().

typedef struct {
    float ranges[TOF_SENSOR_COUNT * TOF_SENSOR_RESO];
} tof_scan_collapsed_t;

tof_scan_collapsed_t tof_get_collapsed_scan(void);

// Snapshot online/fresh state for every sensor.
tof_health_t tof_get_health(uint32_t max_frame_age_ms);

// True only when all eight sensors are online and have fresh frames.
bool tof_all_sensors_fresh(uint32_t max_frame_age_ms);

// True only when every sensor whose field of view overlaps the requested body
// arc is online and fresh. Angles use the same CW-positive convention as the
// ranging API and may cross 0 degrees. Optional masks aid diagnostics.
bool tof_sector_is_fresh(float angle_min_deg, float angle_max_deg,
                         uint32_t max_frame_age_ms,
                         uint8_t *required_mask, uint8_t *missing_mask);

// Returns the number of sensors that have supplied a complete frame (0–8).
// A successful configure/start command alone is deliberately insufficient.
int tof_sensors_ok_count(void);

/* Every acquired native frame. Timestamps bracket host observation/readout,
 * not the optical integration interval, which the ULD does not timestamp. */
typedef struct {
    uint8_t sensor, streamcount;
    bool changed_during_read;
    uint32_t sequence;
    uint64_t ready_before_us, ready_after_us, read_start_us, read_end_us;
    uint64_t previous_not_ready_us;
    uint16_t transfer_bytes;
    clock_sync_status_t sync;
    int16_t distance_mm[64];
    uint8_t target_status[64];
} tof_record_t;
bool tof_pop_record(tof_record_t *record);
uint32_t tof_record_drops(void);
