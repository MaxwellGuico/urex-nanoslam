#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Command 0x07 on existing command port, no extra UDP ports. */
#define CMD_EXPERIMENT 0x07
#define EXPERIMENT_TAKEOFF_LAND 1
#define EXPERIMENT_RECTANGLE 2
/* Phase is included in telemetry, and every phase transition is recorded by host. */
typedef enum {
    EXP_IDLE, EXP_PRESTREAM, EXP_OFFBOARD, EXP_ARM, EXP_TAKEOFF,
    EXP_DWELL, EXP_LEG, EXP_LAND, EXP_DONE, EXP_ABORT, EXP_REJECTED,
    EXP_PILOT_TAKEOVER
} experiment_phase_t;
typedef struct { uint32_t id; uint8_t mode; float width, length, altitude, speed; } experiment_request_t;
typedef struct { uint32_t id; uint8_t phase, corner; } experiment_status_t;
void experiment_init(void);
bool experiment_submit(const experiment_request_t *request);
void experiment_task(void *arg);
bool experiment_controls_enabled(void);
experiment_status_t experiment_status(void);
