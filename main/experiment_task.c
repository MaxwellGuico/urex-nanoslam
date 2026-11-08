#include "experiment_task.h"
#include "experiment_plan.h"
#include "mavlink_task.h"
#include "tof_task.h"
#include "wifi_task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>
static QueueHandle_t s_requests;
static SemaphoreHandle_t s_mutex;
static experiment_status_t s_status;
static bool s_controls;
static const char *TAG="experiment";
static void phase(uint8_t value, uint8_t corner)
{
    xSemaphoreTake(s_mutex,portMAX_DELAY);
    s_status.phase=value; s_status.corner=corner;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG,"mission=%lu phase=%u corner=%u",(unsigned long)s_status.id,value,corner);
}
void experiment_init(void)
{
    s_mutex=xSemaphoreCreateMutex();
    s_requests=xQueueCreate(1,sizeof(experiment_request_t));
    configASSERT(s_mutex && s_requests);
}
static void controls(bool enabled)
{
    xSemaphoreTake(s_mutex,portMAX_DELAY); s_controls=enabled; xSemaphoreGive(s_mutex);
}
bool experiment_controls_enabled(void)
{
    xSemaphoreTake(s_mutex,portMAX_DELAY); bool out=s_controls; xSemaphoreGive(s_mutex); return out;
}
experiment_status_t experiment_status(void)
{
    xSemaphoreTake(s_mutex,portMAX_DELAY); experiment_status_t out=s_status; xSemaphoreGive(s_mutex); return out;
}
bool experiment_submit(const experiment_request_t *r)
{
#if CONFIG_EXPERIMENT_MISSION_ENABLED && !CONFIG_BENCH_MODE
    if (!r->id || (r->mode!=1 && r->mode!=2 && r->mode!=3) ||
        !isfinite(r->width)|| !isfinite(r->length)|| !isfinite(r->altitude)|| !isfinite(r->speed)||
        r->width<0.2f || r->width>10 || r->length<0.2f || r->length>10 ||
        r->altitude<0.3f || r->altitude>2 || r->speed<0.05f || r->speed>0.5f) return false;
    experiment_status_t status=experiment_status();
    if (r->id==status.id) return true; // UDP retries cannot restart the same mission.
    if (experiment_controls_enabled() || mavlink_get_state().armed) return false;
    return xQueueSend(s_requests,r,0)==pdTRUE;
#else
    (void)r; return false;
#endif
}
static bool healthy(drone_state_t d, uint32_t now)
{
    return isfinite(d.x) && isfinite(d.y) && isfinite(d.z) && isfinite(d.heading) &&
        isfinite(d.vx) && isfinite(d.vy) && isfinite(d.vz) &&
        d.last_pos_ms && now-d.last_pos_ms<250 && d.last_att_ms &&
        now-d.last_att_ms<250 && d.last_hb_ms && now-d.last_hb_ms<1500 &&
        mavlink_clock_status().locked
#if !CONFIG_EXPERIMENT_WITHOUT_TOF
        && tof_all_sensors_fresh(TOF_NAV_MAX_FRAME_AGE_MS)
#endif
        ;
}
void experiment_task(void *arg)
{
    (void)arg;
    for (;;) {
        experiment_request_t request;
        xQueueReceive(s_requests,&request,portMAX_DELAY);
        xSemaphoreTake(s_mutex,portMAX_DELAY); s_status.id=request.id; xSemaphoreGive(s_mutex);
        drone_state_t home=mavlink_get_state();
        uint32_t now=(uint32_t)(esp_timer_get_time()/1000);
        if (home.armed || !healthy(home,now)) { phase(EXP_REJECTED,0); continue; }
        wifi_clear_land_request(); wifi_clear_start_request();
        mavlink_set_hold(); controls(true);
        uint8_t stage=EXP_PRESTREAM, corner=0;
        uint32_t began=now, last_action=0, settled=0, last_tick=now;
        float target_z=home.z-request.altitude;
        float command_z=home.z;
        experiment_xy_t command={0,0};
        bool aborted=false, was_armed=false, land_engaged=false;
        phase(stage,corner);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(20));
            now=(uint32_t)(esp_timer_get_time()/1000);
            float dt=fminf((now-last_tick)/1000.f,0.1f); last_tick=now;
            drone_state_t d=mavlink_get_state();
            if (!was_armed && d.armed && (stage==EXP_PRESTREAM || stage==EXP_OFFBOARD)) {
                controls(false); phase(EXP_PILOT_TAKEOVER,corner); break;
            }
            if (stage==EXP_ARM && d.custom_main_mode!=PX4_MAIN_MODE_OFFBOARD) {
                controls(false); phase(EXP_PILOT_TAKEOVER,corner); break;
            }
            // An explicit pilot mode change ends ownership; never switch back to offboard.
            if (was_armed && d.last_hb_ms && now-d.last_hb_ms<1500 &&
                d.armed && d.custom_main_mode!=PX4_MAIN_MODE_OFFBOARD && stage!=EXP_LAND) {
                controls(false); phase(EXP_PILOT_TAKEOVER,corner); break;
            }
            if (was_armed && !d.armed && stage!=EXP_LAND) {
                controls(false); phase(EXP_ABORT,corner); break;
            }
            if (stage!=EXP_LAND && (wifi_land_requested() || !healthy(d,now))) {
                aborted=true;
                if (!d.armed) { controls(false); phase(EXP_ABORT,corner); break; }
                stage=EXP_LAND; began=now; last_action=0; phase(stage,corner);
            }
            // Optional: a rectangle aborts on an obstacle, rather than silently
            // distorting the path. Off by default (CONFIG_EXPERIMENT_OBSTACLE_ABORT).
#if !CONFIG_EXPERIMENT_WITHOUT_TOF && CONFIG_EXPERIMENT_OBSTACLE_ABORT
            if ((stage==EXP_LEG || stage==EXP_DWELL) &&
                tof_get_min_range_in_sector(0,359.9f)<0.4f) {
                aborted=true; stage=EXP_LAND; began=now; last_action=0; phase(stage,corner);
            }
#endif
            if (stage==EXP_PRESTREAM && now-began>=2000) {
                stage=EXP_OFFBOARD; began=now; last_action=0; phase(stage,corner);
            }
            if (stage==EXP_OFFBOARD) {
                if (d.custom_main_mode==PX4_MAIN_MODE_OFFBOARD) {
                    stage=EXP_ARM; began=now; last_action=0; phase(stage,corner);
                } else if (now-began>8000) { controls(false); phase(EXP_ABORT,corner); break; }
                else if (!last_action || now-last_action>=500) { mavlink_set_offboard_mode(); last_action=now; }
            }
            if (stage==EXP_ARM) {
                if (d.armed) { was_armed=true; stage=EXP_TAKEOFF; began=now; phase(stage,corner); }
                else if (now-began>8000) { controls(false); phase(EXP_ABORT,corner); break; }
                else if (!last_action || now-last_action>=500) { mavlink_arm(true); last_action=now; }
            }
            if (stage==EXP_TAKEOFF) {
                command_z=fmaxf(target_z,command_z-0.2f*dt);
                mavlink_set_position_ned(home.x,home.y,command_z,home.heading);
                if (fabsf(d.z-target_z)<0.1f && fabsf(d.vz)<0.08f) {
                    if (!settled) settled=now;
                    if (now-settled>=1000) { stage=EXP_DWELL; began=now; settled=0; phase(stage,corner); }
                } else settled=0;
                if (now-began>20000) { aborted=true; stage=EXP_LAND; began=now; last_action=0; phase(stage,corner); }
            }
            if (stage==EXP_DWELL) {
                experiment_xy_t goal=experiment_waypoint(request.mode,corner,request.length,request.width);
                mavlink_set_position_ned(home.x+goal.x,home.y+goal.y,target_z,home.heading);
                float error=hypotf(d.x-home.x-goal.x,d.y-home.y-goal.y);
                bool stable=experiment_settled(error,hypotf(d.vx,d.vy),0.1f) &&
                            fabsf(d.z-target_z)<=0.1f && fabsf(d.vz)<=0.08f;
                if (!stable) settled=0;
                else if (!settled) settled=now;
                if (settled && now-settled>=3000) {
                    settled=0;
                    if (corner+1u>=experiment_waypoint_count(request.mode)) {
                        stage=EXP_LAND; last_action=0;
                    } else { ++corner; stage=EXP_LEG; }
                    began=now; phase(stage,corner);
                } else if (now-began>30000) { aborted=true; stage=EXP_LAND; began=now; last_action=0; phase(stage,corner); }
            }
            if (stage==EXP_LEG) {
                experiment_xy_t goal=experiment_waypoint(request.mode,corner,request.length,request.width);
                // Bound setpoint lead if the aircraft lags; requested speed is a setpoint limit.
                if (hypotf(command.x-(d.x-home.x),command.y-(d.y-home.y))<0.25f)
                    command=experiment_step(command,goal,request.speed,dt);
                mavlink_set_position_ned(home.x+command.x,home.y+command.y,target_z,home.heading);
                float error=hypotf(d.x-home.x-goal.x,d.y-home.y-goal.y);
                if (experiment_settled(error,hypotf(d.vx,d.vy),0.1f) &&
                    hypotf(command.x-goal.x,command.y-goal.y)<0.001f) {
                    stage=EXP_DWELL; began=now; settled=0; phase(stage,corner);
                }
                if (now-began>120000) { aborted=true; stage=EXP_LAND; began=now; last_action=0; phase(stage,corner); }
            }
            if (stage==EXP_LAND) {
                // Stop offboard setpoints once requesting PX4's autonomous land mode.
                controls(false);
                if (!d.armed) { phase(aborted?EXP_ABORT:EXP_DONE,corner); break; }
                bool px4_landing = d.custom_main_mode==PX4_MAIN_MODE_AUTO &&
                                   d.custom_sub_mode==PX4_SUB_MODE_AUTO_LAND;
                if (px4_landing) {
                    land_engaged=true;   // PX4 owns the landing; stop re-sending
                } else if (land_engaged) {
                    // Mode changed after PX4 accepted LAND (pilot or PX4 failsafe).
                    // Never fight it by re-commanding LAND.
                    phase(EXP_PILOT_TAKEOVER,corner); break;
                } else if (!last_action || now-last_action>=1000) {
                    mavlink_send_land_command(); last_action=now;
                }
                if (now-began>30000) { phase(EXP_ABORT,corner); break; }
            }
        }
    }
}
