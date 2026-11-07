#include "mavlink_task.h"
#include "experiment_task.h"

// MAVLink — header-only, common dialect covers everything PX4 needs.
// Do NOT define MAVLINK_USE_CONVENIENCE_FUNCTIONS — that path requires a
// user-provided mavlink_system global and comm_send_ch() callback.
// We use mavlink_msg_to_send_buffer() directly instead.
#include "common/mavlink.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#ifndef CONFIG_EXPERIMENT_MISSION_ENABLED
#define CONFIG_EXPERIMENT_MISSION_ENABLED 0
#endif
#ifndef CONFIG_NANOLITE_BENCH_MODE
#define CONFIG_NANOLITE_BENCH_MODE 0
#endif

static const char *TAG = "mav";

// ---------------------------------------------------------------------------
// UART config
// ---------------------------------------------------------------------------
#define UART_TX_BUF     1024    // outbound ring buffer
#define UART_RX_BUF     4096    // inbound ring buffer (headroom for QGC bursts)

// ---------------------------------------------------------------------------
// MAVLink UDP bridge for QGroundControl
// ---------------------------------------------------------------------------
// The ESP32 is the only reader of the PX4 UART, so it also owns the bridge:
// raw bytes read from PX4 are forwarded to QGC, and raw UDP packets from QGC
// are written back to PX4.
//   CONFIG_MAVROS_BRIDGE_IN_PORT   = ESP32 UDP listen/source port (QGC target port)
//   CONFIG_MAVROS_BRIDGE_OUT_PORT  = QGC UDP listen port
//   CONFIG_HOST_IPV4_ADDR          = laptop/QGC IPv4 address
#define MAV_BRIDGE_RX_BUF      1024
#define MAV_BRIDGE_MAX_DRAIN   8       // bound UDP packets handled per loop

static int s_bridge_sock = -1;
static struct sockaddr_in s_qgc_addr;
static uint32_t s_bridge_px4_to_qgc_bytes;
static uint32_t s_bridge_qgc_to_px4_bytes;
static uint32_t s_bridge_last_log_ms;

// Pose telemetry required by NanoSLAM-Lite. PX4 may not publish these messages
// continuously on every MAVLink port configuration, so request them explicitly
// and retry only while either stream is stale.
#define POSE_STREAM_INTERVAL_US       (1000000U / CONFIG_NANOLITE_POSE_RATE_HZ)
#define POSE_STREAM_STALE_MS          1000U
#define POSE_STREAM_REQUEST_RETRY_MS  5000U

// ---------------------------------------------------------------------------
// SET_POSITION_TARGET_LOCAL_NED type_mask constants
//
// Bit SET   = ignore this field
// Bit CLEAR = use this field
//
// Bit positions:
//   0-2  : position  (x, y, z)
//   3-5  : velocity  (vx, vy, vz)
//   6-8  : accel     (ax, ay, az)
//   10   : yaw
//   11   : yaw_rate
// ---------------------------------------------------------------------------

// Velocity-only: use vx,vy,vz + yaw_rate — ignore position, accel, yaw
// Bits set: pos(0-2)=0x007, accel(6-8)=0x1C0, yaw(10)=0x400
// Bits clear: vel(3-5), yaw_rate(11)
#define TYPEMASK_VELOCITY       (uint16_t)(0x007 | 0x1C0 | 0x400)   // 0x5C7

// Position-only: use x,y,z + yaw — ignore velocity, accel, yaw_rate
// Bits set: vel(3-5)=0x038, accel(6-8)=0x1C0, yaw_rate(11)=0x800
// Bits clear: pos(0-2), yaw(10)
#define TYPEMASK_POSITION       (uint16_t)(0x038 | 0x1C0 | 0x800)   // 0x9F8

// Mixed XY-velocity + Z-position + yaw-position:
//   Use: vx(3), vy(4), z(2), yaw(10)  → bits clear
//   Ignore: x(0), y(1), vz(5), accel(6-8), yaw_rate(11)  → bits set
#define TYPEMASK_VEL_XY_POS_Z  (uint16_t)(0x001 | 0x002 | 0x020 | 0x1C0 | 0x800)  // 0x9E3

// ---------------------------------------------------------------------------
// PX4 custom modes (needed for MAV_CMD_DO_SET_MODE)
// ---------------------------------------------------------------------------
#define PX4_CUSTOM_MAIN_MODE_OFFBOARD   6

// ---------------------------------------------------------------------------
// Internal setpoint state
// ---------------------------------------------------------------------------
typedef enum {
    SP_VELOCITY,
    SP_POSITION,
    SP_VEL_XY_POS_Z, // XY velocity + Z position + yaw_rate
    SP_HOLD           // position hold at last known location
} sp_type_t;

typedef struct {
    sp_type_t type;
    float x,  y,  z;           // NED metres (position mode)
    float vx, vy, vz;          // NED m/s    (velocity mode)
    float yaw;                  // radians    (position mode, CW from North)
    float yaw_rate;             // rad/s      (velocity mode)
} setpoint_t;

// ---------------------------------------------------------------------------
// Shared state — protected by mutexes
// ---------------------------------------------------------------------------
static setpoint_t    s_sp;
static drone_state_t s_state;
static uint32_t      s_sp_update_ms;   // timestamp of last setpoint update

static SemaphoreHandle_t s_sp_mutex;
static SemaphoreHandle_t s_state_mutex;

static clock_sync_filter_t s_clock;
static int64_t s_timesync_sent_us;
static uint32_t s_pos_rx_count;
static uint32_t s_att_rx_count;
static uint32_t s_hb_rx_count;

// If no task updates the setpoint for this long, auto-switch to position hold.
// Protects against nav_task crash leaving a stale velocity command.
#define SP_STALE_TIMEOUT_MS  300

// ---------------------------------------------------------------------------
// MAVLink UDP bridge helpers
// ---------------------------------------------------------------------------

static void mavlink_bridge_init(void)
{
    s_bridge_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_bridge_sock < 0) {
        ESP_LOGE(TAG, "QGC bridge socket create failed: errno=%d", errno);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(CONFIG_MAVROS_BRIDGE_IN_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_bridge_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "QGC bridge bind failed on UDP %d: errno=%d",
                 CONFIG_MAVROS_BRIDGE_IN_PORT, errno);
        close(s_bridge_sock);
        s_bridge_sock = -1;
        return;
    }

    int flags = fcntl(s_bridge_sock, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(s_bridge_sock, F_SETFL, flags | O_NONBLOCK);
    }

    memset(&s_qgc_addr, 0, sizeof(s_qgc_addr));
    s_qgc_addr.sin_family = AF_INET;
    s_qgc_addr.sin_port   = htons(CONFIG_MAVROS_BRIDGE_OUT_PORT);
    if (inet_aton(CONFIG_HOST_IPV4_ADDR, &s_qgc_addr.sin_addr) == 0) {
        ESP_LOGE(TAG, "Invalid QGC host IPv4 address: %s", CONFIG_HOST_IPV4_ADDR);
        close(s_bridge_sock);
        s_bridge_sock = -1;
        return;
    }

    ESP_LOGI(TAG, "QGC MAVLink bridge enabled: PX4 UART <-> UDP %s:%d, local UDP %d",
             CONFIG_HOST_IPV4_ADDR, CONFIG_MAVROS_BRIDGE_OUT_PORT,
             CONFIG_MAVROS_BRIDGE_IN_PORT);
}

static inline void bridge_forward_px4_to_qgc(const uint8_t *buf, int len)
{
    if (s_bridge_sock < 0 || len <= 0) return;
    int sent = sendto(s_bridge_sock, buf, len, MSG_DONTWAIT,
                      (const struct sockaddr *)&s_qgc_addr, sizeof(s_qgc_addr));
    if (sent > 0) s_bridge_px4_to_qgc_bytes += (uint32_t)sent;
}

static void bridge_poll_qgc_to_px4(void)
{
    if (s_bridge_sock < 0) return;
    static uint8_t buf[MAV_BRIDGE_RX_BUF];   // static: keep it off the task stack

    // Bounded drain so the setpoint stream is never starved by QGC bursts
    // (e.g. a full parameter download).
    for (int i = 0; i < MAV_BRIDGE_MAX_DRAIN; i++) {
        int len = recvfrom(s_bridge_sock, buf, sizeof(buf), 0, NULL, NULL);
        if (len <= 0) break;
        int written = uart_write_bytes(MAV_UART_PORT, (const char *)buf, len);
        if (written > 0) s_bridge_qgc_to_px4_bytes += (uint32_t)written;
    }
}

static void bridge_periodic_log(uint32_t now_ms)
{
    if (s_bridge_sock < 0) return;
    if ((uint32_t)(now_ms - s_bridge_last_log_ms) >= 5000U) {
        ESP_LOGI(TAG, "QGC bridge traffic: PX4->QGC=%lu bytes, QGC->PX4=%lu bytes",
                 (unsigned long)s_bridge_px4_to_qgc_bytes,
                 (unsigned long)s_bridge_qgc_to_px4_bytes);
        s_bridge_last_log_ms = now_ms;
    }
}


// ---------------------------------------------------------------------------
// MAVLink send helpers
// ---------------------------------------------------------------------------

static void uart_send_buf(const uint8_t *buf, uint16_t len)
{
    uart_write_bytes(MAV_UART_PORT, (const char *)buf, len);
}

// Send a pre-populated mavlink_message_t over UART
static void send_message(mavlink_message_t *msg)
{
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_send_buf(buf, len);
}

// ---------------------------------------------------------------------------
// 1 Hz: HEARTBEAT
// Tells PX4 a companion computer is alive.  Type = onboard controller.
// ---------------------------------------------------------------------------
static void send_heartbeat(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,    // we are a companion computer
        MAV_AUTOPILOT_INVALID,          // not an autopilot
        0,                              // base_mode (not used)
        0,                              // custom_mode (not used)
        MAV_STATE_ACTIVE
    );
    send_message(&msg);
}

static void request_message_interval(uint32_t message_id, uint32_t interval_us)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        PX4_SYSID,
        PX4_COMPID,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        (float)message_id,
        (float)interval_us,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f
    );
    send_message(&msg);
}

static void request_pose_streams(void)
{
    request_message_interval(MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                             POSE_STREAM_INTERVAL_US);
    request_message_interval(MAVLINK_MSG_ID_ATTITUDE,
                             POSE_STREAM_INTERVAL_US);
    request_message_interval(MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
                             POSE_STREAM_INTERVAL_US);
    ESP_LOGI(TAG, "Requested PX4 position + attitude at %d Hz",
             CONFIG_NANOLITE_POSE_RATE_HZ);
}

// ---------------------------------------------------------------------------
// 20 Hz: SET_POSITION_TARGET_LOCAL_NED
//
// This is the offboard keepalive.  PX4 drops offboard mode if this message
// stops arriving for ~500ms.  We must send it every 50ms without fail.
// ---------------------------------------------------------------------------
static void send_setpoint(void)
{
    // Snapshot under mutex — copy is cheap (28 bytes)
    xSemaphoreTake(s_sp_mutex, portMAX_DELAY);
    setpoint_t sp = s_sp;
    uint32_t sp_age_ms = (uint32_t)(esp_timer_get_time() / 1000) - s_sp_update_ms;
    xSemaphoreGive(s_sp_mutex);

    // Safety watchdog: if a velocity setpoint hasn't been refreshed, auto-hold.
    // This catches nav_task crashes that would leave a stale velocity command.
    // Position/hold setpoints are inherently safe (drone stays in place).
    if ((sp.type == SP_VELOCITY || sp.type == SP_VEL_XY_POS_Z)
            && sp_age_ms > SP_STALE_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Setpoint stale (%lu ms) — auto hold", (unsigned long)sp_age_ms);
        mavlink_set_hold();
        // Re-snapshot after auto-hold
        xSemaphoreTake(s_sp_mutex, portMAX_DELAY);
        sp = s_sp;
        xSemaphoreGive(s_sp_mutex);
    }

    mavlink_message_t msg;

    if (sp.type == SP_VELOCITY) {
        // Velocity mode: position fields MUST be NaN, not 0.
        // yaw field MUST be NaN — 0.0 commands North-facing (PX4 bug).
        mavlink_msg_set_position_target_local_ned_pack(
            OBC_SYSID,
            OBC_COMPID,
            &msg,
            (uint32_t)(esp_timer_get_time() / 1000),    // time_boot_ms
            PX4_SYSID,
            PX4_COMPID,
            MAV_FRAME_LOCAL_NED,
            TYPEMASK_VELOCITY,
            NAN, NAN, NAN,              // position  — ignored (type_mask)
            sp.vx, sp.vy, sp.vz,        // velocity  — used
            NAN, NAN, NAN,              // accel     — ignored
            NAN,                        // yaw       — ignored (MUST be NaN)
            sp.yaw_rate                 // yaw_rate  — used
        );

    } else if (sp.type == SP_VEL_XY_POS_Z) {
        // Mixed mode: XY velocity + Z position + yaw position.
        // Unused position axes (x,y) and velocity axis (vz) MUST be NaN.
        mavlink_msg_set_position_target_local_ned_pack(
            OBC_SYSID,
            OBC_COMPID,
            &msg,
            (uint32_t)(esp_timer_get_time() / 1000),
            PX4_SYSID,
            PX4_COMPID,
            MAV_FRAME_LOCAL_NED,
            TYPEMASK_VEL_XY_POS_Z,
            NAN, NAN, sp.z,             // x,y ignored; z used
            sp.vx, sp.vy, NAN,          // vx,vy used; vz ignored
            NAN, NAN, NAN,              // accel    — ignored
            sp.yaw,                     // yaw      — used
            NAN                         // yaw_rate — ignored
        );

    } else {
        // Position / hold mode: velocity and yaw_rate fields MUST be NaN.
        float yaw = (sp.type == SP_HOLD) ? NAN : sp.yaw;
        mavlink_msg_set_position_target_local_ned_pack(
            OBC_SYSID,
            OBC_COMPID,
            &msg,
            (uint32_t)(esp_timer_get_time() / 1000),
            PX4_SYSID,
            PX4_COMPID,
            MAV_FRAME_LOCAL_NED,
            TYPEMASK_POSITION,
            sp.x, sp.y, sp.z,           // position  — used
            NAN, NAN, NAN,              // velocity  — ignored (MUST be NaN)
            NAN, NAN, NAN,              // accel     — ignored
            yaw,                        // yaw       — used (or NaN = keep current)
            NAN                         // yaw_rate  — ignored (MUST be NaN)
        );
    }

    send_message(&msg);
}

// ---------------------------------------------------------------------------
// Command: arm / disarm
// ---------------------------------------------------------------------------
static void send_arm_disarm(bool arm)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        PX4_SYSID,
        PX4_COMPID,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,              // confirmation
        arm ? 1.0f : 0.0f,  // param1: 1=arm, 0=disarm
        0, 0, 0, 0, 0, 0    // unused params
    );
    send_message(&msg);
    if (arm) {
        ESP_LOGI(TAG, "ARM sent");
    } else {
        ESP_LOGI(TAG, "DISARM sent");
    }
}

// ---------------------------------------------------------------------------
// Command: switch to OFFBOARD mode
// ---------------------------------------------------------------------------
static void send_offboard_mode(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        PX4_SYSID,
        PX4_COMPID,
        MAV_CMD_DO_SET_MODE,
        0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,   // param1: use custom mode
        PX4_CUSTOM_MAIN_MODE_OFFBOARD,       // param2: custom_mode = offboard
        0, 0, 0, 0, 0                        // unused
    );
    send_message(&msg);
    ESP_LOGI(TAG, "OFFBOARD mode command sent");
}

// ---------------------------------------------------------------------------
// Command: land
// ---------------------------------------------------------------------------
static void send_land(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        PX4_SYSID,
        PX4_COMPID,
        MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,         // unused: abort alt, precision land, empty, yaw
        NAN, NAN,            // lat/lon: NaN = land in place
        NAN                  // altitude
    );
    send_message(&msg);
    ESP_LOGI(TAG, "LAND command sent");
}

// ---------------------------------------------------------------------------
// Incoming message parser
// Drains whatever is in the UART RX buffer and updates s_state.
// ---------------------------------------------------------------------------
static void parse_incoming(void)
{
    uint8_t byte;
    mavlink_message_t msg;
    static mavlink_status_t status;

    // Read the available bytes in chunks without blocking. Each chunk is
    // forwarded raw to QGC, then parsed byte-by-byte for local state.
    uint8_t rx_buf[256];
    int rx_len = 0, rx_pos = 0;
    size_t remaining = 0;
    uart_get_buffered_data_len(MAV_UART_PORT, &remaining);

    while (rx_pos < rx_len || remaining > 0) {
        if (rx_pos >= rx_len) {
            size_t want = remaining > sizeof(rx_buf) ? sizeof(rx_buf) : remaining;
            int n = uart_read_bytes(MAV_UART_PORT, rx_buf, want, 0);
            if (n <= 0) break;
            bridge_forward_px4_to_qgc(rx_buf, n);
            remaining -= (size_t)n;
            rx_len = n;
            rx_pos = 0;
        }
        byte = rx_buf[rx_pos++];

        if (!mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) continue;

        // Only process messages from PX4 (sysid=1, compid=1)
        if (msg.sysid != PX4_SYSID || msg.compid != PX4_COMPID) continue;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        switch (msg.msgid) {

        case MAVLINK_MSG_ID_TIMESYNC: {
            mavlink_timesync_t ts;
            mavlink_msg_timesync_decode(&msg, &ts);
            int64_t rx_us = esp_timer_get_time();
            /* Replies are matched on our exact outstanding nonce only. Some PX4
             * builds leave target_system/target_component uninitialised in
             * TIMESYNC replies (observed 148/108 on this vehicle), so the target
             * fields cannot be trusted for replies. */
            if (ts.tc1 > 0) {
                if (s_timesync_sent_us <= 0 || ts.ts1 != s_timesync_sent_us * 1000) break;
                xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                clock_sync_observe(&s_clock, s_timesync_sent_us, rx_us, ts.tc1 / 1000);
                xSemaphoreGive(s_state_mutex);
                s_timesync_sent_us = 0;
                break;
            }
            /* Request from PX4 (tc1 == 0): answer it if addressed to us or broadcast. */
            if ((ts.target_system && ts.target_system != OBC_SYSID) ||
                (ts.target_component && ts.target_component != OBC_COMPID)) break;
            if (ts.tc1 == 0) {
                mavlink_message_t reply;
                mavlink_msg_timesync_pack(OBC_SYSID, OBC_COMPID, &reply,
                                         rx_us * 1000, ts.ts1, msg.sysid, msg.compid);
                send_message(&reply);
            }
            break;
        }

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t pos;
            mavlink_msg_local_position_ned_decode(&msg, &pos);

            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.x  = pos.x;   s_state.y  = pos.y;   s_state.z  = pos.z;
            s_state.vx = pos.vx;  s_state.vy = pos.vy;  s_state.vz = pos.vz;
            s_state.enu_z     = -pos.z;   // NED z-down → ENU z-up
            s_state.pos_boot_ms = pos.time_boot_ms;
            s_state.last_pos_ms = now_ms;
            xSemaphoreGive(s_state_mutex);
            ++s_pos_rx_count;
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg, &att);

            // Wrap from (-π, π] to [0, 2π) for internal use.
            float heading = att.yaw;
            if (heading < 0.0f) heading += 2.0f * (float)M_PI;

            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.heading     = heading;
            s_state.roll        = att.roll;
            s_state.pitch       = att.pitch;
            // Use ATTITUDE for rates/quaternion only while no ATTITUDE_QUATERNION
            // has arrived in the last 500 ms.
            if (s_state.last_q_ms == 0 || now_ms - s_state.last_q_ms > 500) {
                s_state.rollspeed  = att.rollspeed;
                s_state.pitchspeed = att.pitchspeed;
                s_state.yawspeed   = att.yawspeed;
                float cr = cosf(att.roll * 0.5f),  sr = sinf(att.roll * 0.5f);
                float cp = cosf(att.pitch * 0.5f), sp = sinf(att.pitch * 0.5f);
                float cy = cosf(att.yaw * 0.5f),   sy = sinf(att.yaw * 0.5f);
                s_state.q[0] = cr * cp * cy + sr * sp * sy;
                s_state.q[1] = sr * cp * cy - cr * sp * sy;
                s_state.q[2] = cr * sp * cy + sr * cp * sy;
                s_state.q[3] = cr * cp * sy - sr * sp * cy;
                s_state.q_from_px4  = 0;
                s_state.att_boot_ms = att.time_boot_ms;
            }
            s_state.last_att_ms = now_ms;
            xSemaphoreGive(s_state_mutex);
            ++s_att_rx_count;
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
            mavlink_attitude_quaternion_t aq;
            mavlink_msg_attitude_quaternion_decode(&msg, &aq);

            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.q[0] = aq.q1;  s_state.q[1] = aq.q2;
            s_state.q[2] = aq.q3;  s_state.q[3] = aq.q4;
            s_state.rollspeed   = aq.rollspeed;
            s_state.pitchspeed  = aq.pitchspeed;
            s_state.yawspeed    = aq.yawspeed;
            s_state.q_from_px4  = 1;
            s_state.att_boot_ms = aq.time_boot_ms;
            s_state.last_q_ms   = now_ms;
            xSemaphoreGive(s_state_mutex);
            break;
        }

        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

            bool armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

            // PX4 custom mode is packed: upper byte = main, lower 3 bytes = sub
            uint8_t main_mode = (uint8_t)((hb.custom_mode >> 16) & 0xFF);
            uint8_t sub_mode  = (uint8_t)((hb.custom_mode >> 24) & 0xFF);

            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.armed             = armed;
            s_state.custom_main_mode  = main_mode;
            s_state.custom_sub_mode   = sub_mode;
            s_state.last_hb_ms        = now_ms;
            xSemaphoreGive(s_state_mutex);
            ++s_hb_rx_count;
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            if (ack.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
                ESP_LOGI(TAG, "PX4 stream request ACK result=%u",
                         (unsigned)ack.result);
            }
            break;
        }

        default:
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// Public API implementations
// ---------------------------------------------------------------------------

void mavlink_task_init(void)
{
    // Zero state
    memset(&s_sp,    0, sizeof(s_sp));
    memset(&s_state, 0, sizeof(s_state));
    clock_sync_reset(&s_clock);
    s_timesync_sent_us=0;
    s_pos_rx_count = 0;
    s_att_rx_count = 0;
    s_hb_rx_count = 0;

    // Default to position hold at origin — safe until mission_task sets a real target
    s_sp.type = SP_HOLD;
    s_sp_update_ms = (uint32_t)(esp_timer_get_time() / 1000);

    // Create mutexes before the task starts
    s_sp_mutex    = xSemaphoreCreateMutex();
    s_state_mutex = xSemaphoreCreateMutex();

    configASSERT(s_sp_mutex    != NULL);
    configASSERT(s_state_mutex != NULL);
}

void mavlink_set_velocity_ned(float vx, float vy, float vz, float yaw_rate)
{
    xSemaphoreTake(s_sp_mutex, portMAX_DELAY);
    s_sp.type     = SP_VELOCITY;
    s_sp.vx       = vx;
    s_sp.vy       = vy;
    s_sp.vz       = vz;
    s_sp.yaw_rate = yaw_rate;
    s_sp_update_ms = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreGive(s_sp_mutex);
}

void mavlink_set_velocity_xy_position_z(float vx, float vy, float z, float yaw)
{
    xSemaphoreTake(s_sp_mutex, portMAX_DELAY);
    s_sp.type = SP_VEL_XY_POS_Z;
    s_sp.vx   = vx;
    s_sp.vy   = vy;
    s_sp.z    = z;
    s_sp.yaw  = yaw;
    s_sp_update_ms = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreGive(s_sp_mutex);
}

void mavlink_set_position_ned(float x, float y, float z, float yaw)
{
    xSemaphoreTake(s_sp_mutex, portMAX_DELAY);
    s_sp.type = SP_POSITION;
    s_sp.x    = x;
    s_sp.y    = y;
    s_sp.z    = z;
    s_sp.yaw  = yaw;
    s_sp_update_ms = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreGive(s_sp_mutex);
}

void mavlink_set_hold(void)
{
    // Capture current NED position and freeze there
    drone_state_t st = mavlink_get_state();

    xSemaphoreTake(s_sp_mutex, portMAX_DELAY);
    s_sp.type = SP_HOLD;
    s_sp.x    = st.x;
    s_sp.y    = st.y;
    s_sp.z    = st.z;
    s_sp_update_ms = (uint32_t)(esp_timer_get_time() / 1000);
    // Yaw set to NaN in send_setpoint() for SP_HOLD — PX4 keeps current heading
    xSemaphoreGive(s_sp_mutex);
}

void mavlink_arm(bool arm)
{
    send_arm_disarm(arm);
}

void mavlink_set_offboard_mode(void)
{
    send_offboard_mode();
}

void mavlink_send_land_command(void)
{
    send_land();
}

clock_sync_status_t mavlink_clock_status(void)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    clock_sync_status_t copy = clock_sync_status(&s_clock, esp_timer_get_time());
    xSemaphoreGive(s_state_mutex);
    return copy;
}

drone_state_t mavlink_get_state(void)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    drone_state_t copy = s_state;
    xSemaphoreGive(s_state_mutex);
    return copy;
}

bool mavlink_position_valid(void)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    uint32_t last = s_state.last_pos_ms;
    xSemaphoreGive(s_state_mutex);

    if (last == 0) return false;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    return (now - last) < 500;
}


// ---------------------------------------------------------------------------
// FreeRTOS task
// ---------------------------------------------------------------------------

void mavlink_task(void *arg)
{
    // ---- UART init ----
    const uart_config_t uart_cfg = {
        .baud_rate  = MAV_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        MAV_UART_PORT,
        UART_RX_BUF,
        UART_TX_BUF,
        0,      // no event queue needed
        NULL,
        ESP_INTR_FLAG_IRAM   // IRAM ISR: immune to cache miss from WiFi
    ));
    ESP_ERROR_CHECK(uart_param_config(MAV_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(
        MAV_UART_PORT,
        MAV_TX_PIN,
        MAV_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_LOGI(TAG, "UART%d ready: TX=GPIO%d RX=GPIO%d @ %d baud",
             MAV_UART_PORT, MAV_TX_PIN, MAV_RX_PIN, MAV_BAUD_RATE);

    // wifi_task_init() has already run esp_netif_init(), so the socket can be
    // opened before the link is up; sends just fail until WiFi connects.
    // A failure here only disables QGC bridging, not PX4 control.
    mavlink_bridge_init();

    request_pose_streams();

    // ---- Timing state ----
    TickType_t last_wake_tick = xTaskGetTickCount();
    uint32_t   last_hb_ms     = 0;
    uint32_t last_sp_ms = 0, last_sync_ms = 0;
    uint32_t   last_stream_request_ms =
        (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t   last_rx_report_ms = last_stream_request_ms;
    uint32_t   reported_pos_count = 0;
    uint32_t   reported_att_count = 0;
    uint32_t   reported_hb_count = 0;

    // ---- 20Hz loop ----
    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        // 1 Hz: heartbeat
        if (now_ms - last_hb_ms >= 1000) {
            send_heartbeat();
            last_hb_ms = now_ms;
        }

        // Drain UART every millisecond; keep flight setpoints at 20 Hz.
        // Manual/bench images never publish flight-control setpoints.
        if (!CONFIG_NANOLITE_BENCH_MODE &&
            (!CONFIG_EXPERIMENT_MISSION_ENABLED || experiment_controls_enabled()) &&
            now_ms - last_sp_ms >= 50) {
            send_setpoint();
            last_sp_ms = now_ms;
        }
        if (now_ms - last_sync_ms >= 100) {
            mavlink_message_t sync;
            s_timesync_sent_us = esp_timer_get_time();
            mavlink_msg_timesync_pack(OBC_SYSID, OBC_COMPID, &sync,
                                     0, s_timesync_sent_us * 1000, PX4_SYSID, PX4_COMPID);
            send_message(&sync);
            last_sync_ms = now_ms;
        }

        // QGC -> PX4, then PX4 -> (QGC + s_state)
        bridge_poll_qgc_to_px4();
        parse_incoming();
        bridge_periodic_log(now_ms);

        // Incoming timestamps are captured inside parse_incoming(). Resample
        // after parsing so a message received across a millisecond boundary
        // cannot appear to have an unsigned age of UINT32_MAX.
        now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        const drone_state_t state = mavlink_get_state();
        const uint32_t pos_age_ms = state.last_pos_ms == 0
            ? UINT32_MAX : (uint32_t)(now_ms - state.last_pos_ms);
        const uint32_t att_age_ms = state.last_att_ms == 0
            ? UINT32_MAX : (uint32_t)(now_ms - state.last_att_ms);

        if ((pos_age_ms > POSE_STREAM_STALE_MS ||
             att_age_ms > POSE_STREAM_STALE_MS) &&
            (uint32_t)(now_ms - last_stream_request_ms) >=
                POSE_STREAM_REQUEST_RETRY_MS) {
            request_pose_streams();
            last_stream_request_ms = now_ms;
        }

        if ((uint32_t)(now_ms - last_rx_report_ms) >= 5000U) {
            ESP_LOGI(TAG,
                     "RX/5s: position=%lu attitude=%lu heartbeat=%lu "
                     "age=%lu/%lums",
                     (unsigned long)(s_pos_rx_count - reported_pos_count),
                     (unsigned long)(s_att_rx_count - reported_att_count),
                     (unsigned long)(s_hb_rx_count - reported_hb_count),
                     (unsigned long)pos_age_ms,
                     (unsigned long)att_age_ms);
            reported_pos_count = s_pos_rx_count;
            reported_att_count = s_att_rx_count;
            reported_hb_count = s_hb_rx_count;
            last_rx_report_ms = now_ms;
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(1));
    }
}
