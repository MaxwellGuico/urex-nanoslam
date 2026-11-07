"""Run the actual onboard mission C state machine against a deterministic fake PX4.
This verifies control sequencing and timing, not aircraft dynamics or hardware.
"""
from pathlib import Path
import subprocess
import pytest
ROOT=Path(__file__).resolve().parents[2]

@pytest.mark.parametrize('without_tof,obstacle_abort', [(False, True), (False, False), (True, False)])
def test_onboard_mission_state_machine(tmp_path, without_tof, obstacle_abort):
    includes={
    'esp_timer.h':'#include <stdint.h>\nint64_t esp_timer_get_time(void);\n',
    'esp_log.h':'#define ESP_LOGI(...) ((void)0)\n',
    'tof_task.h':'#include <stdbool.h>\n#define TOF_NAV_MAX_FRAME_AGE_MS 250\nbool tof_all_sensors_fresh(unsigned);\nfloat tof_get_min_range_in_sector(float,float);\n',
    'wifi_task.h':'#include <stdbool.h>\nbool wifi_land_requested(void);\nvoid wifi_clear_land_request(void);\nvoid wifi_clear_start_request(void);\n',
    'freertos/FreeRTOS.h':'''#pragma once
#include <assert.h>
#define pdTRUE 1
#define portMAX_DELAY 0xffffffff
#define pdMS_TO_TICKS(x) (x)
#define configASSERT(x) assert(x)
''',
    'freertos/task.h':'void vTaskDelay(unsigned);\n',
    'freertos/queue.h':'''typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned,unsigned);
int xQueueSend(QueueHandle_t,const void *,unsigned);
int xQueueReceive(QueueHandle_t,void *,unsigned);
''',
    'freertos/semphr.h':'''typedef void *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
int xSemaphoreTake(SemaphoreHandle_t,unsigned);
void xSemaphoreGive(SemaphoreHandle_t);
''',
    }
    for name,content in includes.items():
        path=tmp_path/name; path.parent.mkdir(parents=True,exist_ok=True); path.write_text(content)
    # Move just the translation unit so its hardware headers resolve to these fakes.
    mission=tmp_path/'experiment_task.c'; mission.write_text((ROOT/'main/experiment_task.c').read_text())
    simulator=tmp_path/'simulator.c'
    simulator.write_text(r'''
#include "experiment_task.h"
#include "mavlink_task.h"
#include "freertos/FreeRTOS.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static jmp_buf finished;
static drone_state_t d;
static experiment_request_t request;
static int delivered, scenario, land_count, arm_count, mode_count, land_ticks;
static int64_t now_us=1000000;
static unsigned last_phase=255,last_corner=255,dwell_began,dwell_count;
static float min_x=1000,max_x=-1000,min_y=1000,max_y=-1000,min_z=1000;
void *xQueueCreate(unsigned n,unsigned s) {(void)n;(void)s;return (void*)1;}
void *xSemaphoreCreateMutex(void) {return (void*)1;}
int xSemaphoreTake(void *p,unsigned t) {(void)p;(void)t;return 1;}
void xSemaphoreGive(void *p) {(void)p;}
int xQueueSend(void *q,const void *r,unsigned t) {(void)q;(void)t;request=*(const experiment_request_t*)r;return 1;}
int xQueueReceive(void *q,void *r,unsigned t) {
 (void)q;(void)t;
 if(delivered++) longjmp(finished,1);
 memcpy(r,&request,sizeof(request)); return 1;
}
int64_t esp_timer_get_time(void) {return now_us;}
void vTaskDelay(unsigned ms) {
 experiment_status_t status=experiment_status();
 if(last_phase==EXP_DWELL && (status.phase!=last_phase || status.corner!=last_corner)) {
    assert((unsigned)(now_us/1000)-dwell_began>=3000); ++dwell_count;
 }
 if(status.phase==EXP_DWELL && last_phase!=EXP_DWELL) dwell_began=now_us/1000;
 last_phase=status.phase; last_corner=status.corner;
 now_us+=ms*1000;
 assert(now_us<200000000);
 d.last_pos_ms=d.last_att_ms=d.last_hb_ms=now_us/1000;
 if(scenario==6 && status.phase==EXP_PRESTREAM) { d.armed=true; d.custom_main_mode=3; }
 if(scenario==7 && status.phase==EXP_ARM) d.custom_main_mode=3;
 if(scenario==2 && status.phase==EXP_LEG) d.custom_main_mode=3; // pilot takeover
 // PX4 accepted LAND, then the pilot switches mode 3 s later: the ESP must not re-command LAND.
 if(scenario==10 && status.phase==EXP_LAND && land_count && ++land_ticks==150) d.custom_main_mode=3;
}
drone_state_t mavlink_get_state(void) {return d;}
clock_sync_status_t mavlink_clock_status(void) {
 return (clock_sync_status_t){.locked=!(scenario==3 && experiment_status().phase==EXP_LEG),.rtt_us=1000};
}
bool tof_all_sensors_fresh(unsigned age) {
 assert(!CONFIG_EXPERIMENT_WITHOUT_TOF);
 (void)age;return !(scenario==9 && experiment_status().phase==EXP_LEG);
}
float tof_get_min_range_in_sector(float a,float b) {
 assert(!CONFIG_EXPERIMENT_WITHOUT_TOF && CONFIG_EXPERIMENT_OBSTACLE_ABORT);
 (void)a;(void)b; return scenario==4 && experiment_status().phase==EXP_LEG ? .2f:10.f;
}
bool wifi_land_requested(void) {return false;}
void wifi_clear_land_request(void) {}
void wifi_clear_start_request(void) {}
void mavlink_set_hold(void) {}
void mavlink_set_offboard_mode(void) {++mode_count;if(scenario!=5) d.custom_main_mode=6;}
void mavlink_arm(bool armed) {++arm_count;d.armed=armed;}
void mavlink_send_land_command(void) {
 ++land_count;d.custom_main_mode=4;
 if(scenario==10) d.custom_sub_mode=6; else d.armed=false;
}
void mavlink_set_position_ned(float x,float y,float z,float yaw) {
 d.x=x;d.y=y;d.z=z;d.heading=yaw;
 min_x=fminf(min_x,x);max_x=fmaxf(max_x,x);
 min_y=fminf(min_y,y);max_y=fmaxf(max_y,y);min_z=fminf(min_z,z);
}
int main(int argc,char **argv) {
 assert(argc==2);scenario=atoi(argv[1]);
 d=(drone_state_t){.x=10,.y=-3,.z=2,.heading=.4,.last_pos_ms=1000,.last_att_ms=1000,.last_hb_ms=1000};
 if(scenario==8) d.x=NAN;
 experiment_init();
 request=(experiment_request_t){.id=22,.mode=(scenario==1||scenario==10)?1:2,.width=2,.length=3,.altitude=.5,.speed=.3};
 assert(experiment_submit(&request));
 if(!setjmp(finished)) experiment_task(NULL);
 experiment_status_t status=experiment_status();
 assert(!experiment_controls_enabled());
 if(scenario==0 || scenario==1 || (CONFIG_EXPERIMENT_WITHOUT_TOF && (scenario==4 || scenario==9))
    || (!CONFIG_EXPERIMENT_OBSTACLE_ABORT && scenario==4)) {
   assert(status.phase==EXP_DONE && land_count>0 && arm_count>0 && mode_count>0);
   assert(fabsf(d.x-10)<.001f && fabsf(d.y+3)<.001f && fabsf(min_z-1.5)<.001f);
   assert(dwell_count==(scenario==1?1:5));
   if(scenario!=1) {assert(fabsf(max_x-13)<.001f && fabsf(max_y+1)<.001f);}
 } else if(scenario==2 || scenario==6 || scenario==7) {
   assert(status.phase==EXP_PILOT_TAKEOVER && land_count==0);
 } else if(scenario==3 || scenario==4 || scenario==9) {
   assert(status.phase==EXP_ABORT && land_count>0);
 } else if(scenario==8) {
   assert(status.phase==EXP_REJECTED && arm_count==0 && mode_count==0);
 } else if(scenario==10) {
   assert(status.phase==EXP_PILOT_TAKEOVER && land_count==1 && d.armed);
 } else if(scenario==5) {
   assert(status.phase==EXP_ABORT && arm_count==0);
 }
 puts("onboard simulation passed");
 return 0;
}
''')
    binary=tmp_path/'mission'
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Wno-unused-variable',
        '-DCONFIG_EXPERIMENT_MISSION_ENABLED=1','-DCONFIG_NANOLITE_BENCH_MODE=0',
        f'-DCONFIG_EXPERIMENT_WITHOUT_TOF={int(without_tof)}',
        f'-DCONFIG_EXPERIMENT_OBSTACLE_ABORT={int(obstacle_abort)}',
        '-I',str(tmp_path),'-I',str(ROOT/'main'),str(mission),str(simulator),
        str(ROOT/'main/experiment_plan.c'),'-lm','-o',str(binary)],check=True)
    for scenario in range(11):
        subprocess.run([str(binary),str(scenario)],check=True)
