#include "wifi_task.h"
#include "mavlink_task.h"
#include "nav_task.h"
#include "at_detect.h"
#include "odom.h"
#include "tof_task.h"
#include "experiment_task.h"
#include "esp_random.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

static const char *TAG = "wifi";
static uint32_t s_boot_id;
_Static_assert(sizeof(wifi_tof_record_header_t)==88, "Native record header changed");

_Static_assert(sizeof(wifi_telem_pkt_t) == 165, "Telemetry wire layout changed");
_Static_assert(sizeof(wifi_tof_debug_pkt_t) == 203, "ToF debug wire layout changed");

/* Cruise altitude for CMD_GOTO — must match CRUISE_ALT_M in main.c */
#define WIFI_CRUISE_ALT_M   0.5f

static volatile bool s_land_requested  = false;
static volatile bool s_start_requested = false;
static volatile bool s_wifi_connected  = false;

static uint16_t telemetry_age_ms(uint32_t now_ms, uint32_t last_ms)
{
    if (last_ms == 0) return UINT16_MAX;
    uint32_t age = now_ms - last_ms;
    return age > UINT16_MAX ? UINT16_MAX : (uint16_t)age;
}

/* Peer drone positions (map frame), protected by s_peer_mutex */
static wifi_peer_list_t   s_peers = { .count = 0 };
static SemaphoreHandle_t  s_peer_mutex;

static const char *wifi_disconnect_reason(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_NO_AP_FOUND: return "access point not found; check SSID and 2.4 GHz hotspot";
    case WIFI_REASON_AUTH_FAIL: return "authentication failed; check password/security";
    case WIFI_REASON_AUTH_EXPIRE: return "authentication expired";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "security handshake timed out; check password/signal";
    case WIFI_REASON_BEACON_TIMEOUT: return "access point beacons lost; check signal/hotspot";
    case WIFI_REASON_ASSOC_FAIL: return "access point association failed";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY: return "no access point with compatible security";
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD: return "access point security below threshold";
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD: return "access point signal below threshold";
    default: return "see ESP-IDF Wi-Fi reason code";
    }
}

static void wifi_connect(void)
{
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Connection request failed: %s", esp_err_to_name(err));
    }
}

#if CONFIG_WIFI_STATIC_IP
static esp_err_t wifi_configure_static_ip(esp_netif_t *netif)
{
    esp_netif_ip_info_t ip = {0};
    if (esp_netif_str_to_ip4(CONFIG_DRONE_STATIC_IPV4_ADDR, &ip.ip) != ESP_OK ||
        esp_netif_str_to_ip4(CONFIG_WIFI_STATIC_GATEWAY, &ip.gw) != ESP_OK ||
        esp_netif_str_to_ip4(CONFIG_WIFI_STATIC_NETMASK, &ip.netmask) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid static IPv4 address, gateway, or subnet mask in sdkconfig");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_netif_dhcpc_stop(netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return err;
    }
    ESP_LOGI(TAG, "Static IPv4: %s, gateway: %s, mask: %s",
             CONFIG_DRONE_STATIC_IPV4_ADDR, CONFIG_WIFI_STATIC_GATEWAY,
             CONFIG_WIFI_STATIC_NETMASK);
    return esp_netif_set_ip_info(netif, &ip);
}
#endif

/* ---------------------------------------------------------------------------
 * WiFi event handler — auto-reconnect on disconnect
 * --------------------------------------------------------------------------- */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "Associated with access point; waiting for IPv4 readiness");
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *e = data;
        s_wifi_connected = false;
        ESP_LOGW(TAG, "Disconnected: reason=%u (%s) — reconnecting...",
                 (unsigned)e->reason, wifi_disconnect_reason(e->reason));
        wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_wifi_connected = true;
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        s_wifi_connected = false;
        ESP_LOGW(TAG, "IPv4 address lost");
    }
}

/* ---------------------------------------------------------------------------
 * wifi_task_init — start station mode and return immediately
 *
 * Sensor, camera, MAVLink, and SLAM startup must not depend on the availability
 * of the ground-station access point. The event handler keeps retrying in the
 * background and s_wifi_connected becomes true once IPv4 is ready (static or DHCP).
 * --------------------------------------------------------------------------- */
void wifi_task_init(void)
{
    s_boot_id=esp_random();
    s_peer_mutex  = xSemaphoreCreateMutex();
    configASSERT(s_peer_mutex != NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    configASSERT(sta_netif != NULL);
#if CONFIG_WIFI_STATIC_IP
    ESP_ERROR_CHECK(wifi_configure_static_ip(sta_netif));
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid,     CONFIG_WIFI_SSID,     sizeof(wifi_cfg.sta.ssid));
    strncpy((char *)wifi_cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to '%s'...", CONFIG_WIFI_SSID);
}

/* ---------------------------------------------------------------------------
 * wifi_task — 10 Hz telemetry loop
 * --------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------
 * Process an incoming command packet from the laptop
 * --------------------------------------------------------------------------- */
static void handle_command(const wifi_cmd_pkt_t *cmd)
{
#if CONFIG_EXPERIMENT_MISSION_ENABLED
    if (cmd->cmd_type==CMD_LAND || cmd->cmd_type==CMD_HOLD) s_land_requested=true;
    return; // Experiment commands exclusively own flight control in this image.
#endif
    /* Always update the fleet's known-tag list */
    at_detect_set_known_tags(cmd->found_tag_ids, WIFI_MAX_FOUND_TAGS);

    switch (cmd->cmd_type) {
    case CMD_START:
        ESP_LOGI(TAG, "CMD_START");
        s_start_requested = true;
        break;
    case CMD_GOTO:
        nav_set_goal_map(cmd->goal_x, cmd->goal_y, -(WIFI_CRUISE_ALT_M));
        ESP_LOGI(TAG, "CMD_GOTO map(%.2f,%.2f)", cmd->goal_x, cmd->goal_y);
        break;
    case CMD_LAND:
        ESP_LOGI(TAG, "CMD_LAND");
        s_land_requested = true;
        break;
    case CMD_HOLD:
        nav_cancel();
        ESP_LOGI(TAG, "CMD_HOLD");
        break;
    default:
        ESP_LOGW(TAG, "Unknown cmd type 0x%02x", cmd->cmd_type);
        break;
    }
}

/* Parse and apply a CMD_SET_NAV_TAGS packet */
static void handle_nav_tags(const uint8_t *buf, int len)
{
    /* Header: pkt_type(1) + cmd_type(1) + tag_count(1) + start_x(4) + start_y(4) = 11 */
    if (len < 11) return;
    uint8_t tag_count = buf[2];
    if (tag_count > WIFI_MAX_NAV_TAGS) tag_count = WIFI_MAX_NAV_TAGS;

    int expected = 11 + tag_count * (int)sizeof(wifi_nav_tag_entry_t);
    if (len < expected) {
        ESP_LOGW(TAG, "CMD_SET_NAV_TAGS truncated (%d < %d)", len, expected);
        return;
    }

    float start_map_x, start_map_y;
    memcpy(&start_map_x, buf + 3, sizeof(float));
    memcpy(&start_map_y, buf + 7, sizeof(float));
    odom_set_initial_offset(start_map_x, start_map_y);

    const wifi_nav_tag_entry_t *entries =
        (const wifi_nav_tag_entry_t *)(buf + 11);

    nav_tag_t tags[WIFI_MAX_NAV_TAGS];
    for (int i = 0; i < tag_count; i++) {
        tags[i].id = entries[i].id;
        tags[i].x  = entries[i].odom_x;
        tags[i].y  = entries[i].odom_y;
    }
    odom_set_nav_tags(tags, tag_count);
    ESP_LOGI(TAG, "CMD_SET_NAV_TAGS: %d tags, start map(%.2f,%.2f)",
             tag_count, start_map_x, start_map_y);
}

/* Parse and apply a CMD_SET_PEERS packet.
 * Wire format: pkt_type(1) + cmd_type(1) + count(1) + count * (float, float) */
static void handle_peers(const uint8_t *buf, int len)
{
    if (len < 3) return;
    uint8_t count = buf[2];
    if (count > WIFI_MAX_PEERS) count = WIFI_MAX_PEERS;

    int expected = 3 + count * (int)(2 * sizeof(float));
    if (len < expected) {
        ESP_LOGW(TAG, "CMD_SET_PEERS truncated (%d < %d)", len, expected);
        return;
    }

    wifi_peer_list_t list;
    list.count = count;
    const uint8_t *p = buf + 3;
    for (int i = 0; i < count; i++) {
        memcpy(&list.peers[i].map_x, p, sizeof(float)); p += sizeof(float);
        memcpy(&list.peers[i].map_y, p, sizeof(float)); p += sizeof(float);
    }

    list.update_ms = (uint32_t)(esp_timer_get_time() / 1000);

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    s_peers = list;
    xSemaphoreGive(s_peer_mutex);

    ESP_LOGD(TAG, "CMD_SET_PEERS: %d peers", count);
}

wifi_peer_list_t wifi_get_peers(void)
{
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    wifi_peer_list_t copy = s_peers;
    xSemaphoreGive(s_peer_mutex);

    /* Discard peer data older than 1 s — positions are too stale to trust */
    if (copy.count > 0) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now_ms - copy.update_ms) > 1000) {
            copy.count = 0;
        }
    }
    return copy;
}

static void send_tof_records(int sock)
{
    tof_record_t record;
    for (int count=0; count<16 && tof_pop_record(&record); ++count) {
        wifi_tof_record_header_t h={
            .pkt_type=WIFI_PKT_TOF_RECORD, .version=1, .drone_id=CONFIG_DRONE_ID,
            .sensor=record.sensor, .rows=TOF_NATIVE_SIDE, .cols=TOF_NATIVE_SIDE,
            .hz=TOF_RANGING_FREQ_HZ,
            .flags=(record.sync.locked?1:0) | (record.previous_not_ready_us?2:0) | (record.changed_during_read?4:0),
            .boot_id=s_boot_id, .sequence=record.sequence, .queue_drops=tof_record_drops(),
            .ready_before_us=record.ready_before_us, .ready_after_us=record.ready_after_us,
            .read_start_us=record.read_start_us, .read_end_us=record.read_end_us,
            .previous_not_ready_us=record.previous_not_ready_us, .send_us=esp_timer_get_time(),
            .clock_offset_us=record.sync.offset_us,
            .sync_rtt_us=record.sync.rtt_us, .sync_age_us=record.sync.age_us,
            .transfer_bytes=record.transfer_bytes, .streamcount=record.streamcount,
            .sync_samples=record.sync.samples>255?255:record.sync.samples,
        };
        uint8_t buffer[sizeof(h)+TOF_NATIVE_PIXELS*3];
        memcpy(buffer,&h,sizeof(h));
        memcpy(buffer+sizeof(h),record.distance_mm,TOF_NATIVE_PIXELS*2);
        memcpy(buffer+sizeof(h)+TOF_NATIVE_PIXELS*2,record.target_status,TOF_NATIVE_PIXELS);
        send(sock,buffer,sizeof(buffer),MSG_DONTWAIT);
    }
}

/* ---------------------------------------------------------------------------
 * wifi_task — 10 Hz telemetry loop + non-blocking command receive
 * --------------------------------------------------------------------------- */
void wifi_task(void *arg)
{
    /* Telemetry send socket */
    int tx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(tx_sock >= 0);

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(CONFIG_TELEM_PORT),
    };
    inet_aton(CONFIG_HOST_IPV4_ADDR, &dest.sin_addr);
    connect(tx_sock, (struct sockaddr *)&dest, sizeof(dest));

    /* ToF debug send socket */
    int tof_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(tof_sock >= 0);
    struct sockaddr_in tof_dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(WIFI_TOF_DEBUG_PORT),
    };
    inet_aton(CONFIG_HOST_IPV4_ADDR, &tof_dest.sin_addr);
    connect(tof_sock, (struct sockaddr *)&tof_dest, sizeof(tof_dest));

    /* Command receive socket (non-blocking) */
    int rx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(rx_sock >= 0);

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(WIFI_CMD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(rx_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Command socket bind failed (port %d)", WIFI_CMD_PORT);
    }
    int flags = fcntl(rx_sock, F_GETFL, 0);
    fcntl(rx_sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "Telemetry → %s:%d | Commands ← port %d",
             CONFIG_HOST_IPV4_ADDR, CONFIG_TELEM_PORT, WIFI_CMD_PORT);

    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_telem_us=0;
#if CONFIG_TOF_LEGACY_DEBUG && !CONFIG_EXPERIMENT_WITHOUT_TOF
    int64_t last_debug_us=0;
#endif

    while (1) {
        /* ---- Check for incoming commands (non-blocking) ---- */
        {
            uint8_t cmd_buf[256];
            int len = recvfrom(rx_sock, cmd_buf, sizeof(cmd_buf), 0, NULL, NULL);
            if (len >= 2 && cmd_buf[0] == WIFI_PKT_CMD) {
                if (cmd_buf[1] == CMD_EXPERIMENT && len==23) {
                    experiment_request_t r={.mode=cmd_buf[2]};
                    memcpy(&r.id,cmd_buf+3,4);
                    memcpy(&r.width,cmd_buf+7,4); memcpy(&r.length,cmd_buf+11,4);
                    memcpy(&r.altitude,cmd_buf+15,4); memcpy(&r.speed,cmd_buf+19,4);
                    if (!experiment_submit(&r)) ESP_LOGW(TAG,"Experiment command rejected");
                } else if (cmd_buf[1] == CMD_SET_NAV_TAGS) {
                    handle_nav_tags(cmd_buf, len);
                } else if (cmd_buf[1] == CMD_SET_PEERS) {
                    handle_peers(cmd_buf, len);
                } else if (len >= (int)sizeof(wifi_cmd_pkt_t)) {
                    handle_command((const wifi_cmd_pkt_t *)cmd_buf);
                }
            }
        }

        send_tof_records(tof_sock);
        const int64_t tick_us=esp_timer_get_time();
        if (tick_us-last_telem_us < 1000000/CONFIG_DATA_TELEM_HZ) {
            vTaskDelayUntil(&last_wake,pdMS_TO_TICKS(1));
            continue;
        }
        last_telem_us=tick_us;
        /* ---- Snapshot all shared state ---- */
        drone_state_t    drone = mavlink_get_state();
        nav_status_t     ns    = nav_get_status();
        at_detect_pose_t pose  = at_detect_get_pose();

        /* ---- Build telemetry packet ---- */
        wifi_telem_pkt_t pkt = {};
        pkt.pkt_type    = WIFI_PKT_TELEM;
        pkt.drone_id    = CONFIG_DRONE_ID;
        /* Report position in map frame so laptop works in a consistent frame */
        odom_to_map(drone.x, drone.y, &pkt.ned_x, &pkt.ned_y);
        pkt.heading_rad = drone.heading;
        pkt.nav_state   = (uint8_t)ns.state;
        pkt.is_stuck    = (ns.state == NAV_STUCK) ? 1 : 0;

        /* AprilTag — always send the latched tag ID (−1 if none found yet).
         * tag_dist_m comes from live pose when available. */
        pkt.tag_id = at_detect_my_tag_id();
        if (pose.valid) {
            pkt.tag_dist_m = sqrtf(pose.tx * pose.tx + pose.tz * pose.tz);
        } else {
            pkt.tag_dist_m = 0.0f;
        }

        /* VFH blocked state */
        for (int b = 0; b < VFH_BINS; b++)
            pkt.vfh_blocked[b] = ns.vfh_blocked[b] ? 1 : 0;

        pkt.reloc_age_s = odom_reloc_age_s();

        /* Appended PX4 health and kinematics for baseline logging. */
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        pkt.px4_hb_age_ms  = telemetry_age_ms(now_ms, drone.last_hb_ms);
        pkt.px4_pos_age_ms = telemetry_age_ms(now_ms, drone.last_pos_ms);
        pkt.px4_att_age_ms = telemetry_age_ms(now_ms, drone.last_att_ms);
        pkt.px4_link_ok = (pkt.px4_hb_age_ms < 2000
                           && pkt.px4_pos_age_ms < 500
                           && pkt.px4_att_age_ms < 500) ? 1 : 0;
        pkt.px4_armed = drone.armed ? 1 : 0;
        pkt.px4_custom_main_mode = drone.custom_main_mode;
        pkt.ned_z  = drone.z;
        pkt.ned_vx = drone.vx;
        pkt.ned_vy = drone.vy;
        pkt.ned_vz = drone.vz;
        pkt.q_w = drone.q[0];  pkt.q_x = drone.q[1];
        pkt.q_y = drone.q[2];  pkt.q_z = drone.q[3];
        pkt.roll_rate  = drone.rollspeed;
        pkt.pitch_rate = drone.pitchspeed;
        pkt.yaw_rate   = drone.yawspeed;
        pkt.roll_rad   = drone.roll;
        pkt.pitch_rad  = drone.pitch;
        pkt.px4_pos_boot_ms = drone.pos_boot_ms;
        pkt.px4_att_boot_ms = drone.att_boot_ms;
        pkt.q_from_px4 = drone.q_from_px4;

        clock_sync_status_t sync=mavlink_clock_status();
        experiment_status_t exp=experiment_status();
        pkt.esp_send_us=esp_timer_get_time();
        pkt.clock_offset_us=sync.offset_us; pkt.sync_rtt_us=sync.rtt_us;
        pkt.sync_age_us=sync.age_us; pkt.sync_samples=sync.samples;
        pkt.sync_locked=sync.locked; pkt.experiment_phase=exp.phase;
        pkt.mission_id=exp.id; pkt.corner=exp.corner;
#if CONFIG_EXPERIMENT_WITHOUT_TOF
        pkt.tof_side=0; pkt.tof_hz=0;
#else
        pkt.tof_side=TOF_NATIVE_SIDE; pkt.tof_hz=TOF_RANGING_FREQ_HZ;
#endif
#if CONFIG_NANOLITE_BENCH_MODE
        pkt.passive=1;
#endif
        pkt.boot_id=s_boot_id;
        /* ---- Send telemetry ---- */
        send(tx_sock, &pkt, sizeof(pkt), MSG_DONTWAIT);

        /* ---- Send all raw ToF frames, preserving each acquisition time ---- */
#if CONFIG_TOF_LEGACY_DEBUG && !CONFIG_EXPERIMENT_WITHOUT_TOF
        if (TOF_NATIVE_SIDE==8 && tick_us-last_debug_us>=100000) {
            last_debug_us=tick_us;
            tof_scan_t scan = tof_get_scan();
            const uint32_t sent_ms = (uint32_t)(esp_timer_get_time() / 1000);
            for (uint8_t sensor = 0; sensor < TOF_SENSOR_COUNT; ++sensor) {
                wifi_tof_debug_pkt_t dbg = {};
                dbg.pkt_type     = WIFI_PKT_TOF_DEBUG;
                dbg.sensor_idx   = sensor;
                dbg.sensor_ok    = scan.sensor_ok[sensor];
                dbg.timestamp_ms = scan.frame[sensor].timestamp_ms;
                dbg.sent_ms      = sent_ms;
                memcpy(dbg.distance_mm, scan.frame[sensor].distance_mm,
                       sizeof(dbg.distance_mm));
                memcpy(dbg.target_status, scan.frame[sensor].target_status,
                       sizeof(dbg.target_status));
                send(tof_sock, &dbg, sizeof(dbg), MSG_DONTWAIT);
            }
        }

#endif
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

bool wifi_land_requested(void)
{
    return s_land_requested;
}

void wifi_clear_land_request(void)
{
    s_land_requested = false;
}

bool wifi_start_requested(void)
{
    return s_start_requested;
}

void wifi_clear_start_request(void)
{
    s_start_requested = false;
}

bool wifi_is_connected(void)
{
    return s_wifi_connected;
}
