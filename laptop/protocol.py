"""
Packet definitions for ESP32 ↔ laptop UDP protocol.

Telemetry (ESP32 → laptop, configurable snapshot rate):
    165-byte packet with PX4 pose, velocity, sample timestamps, sync, and mission state.
    The parser also accepts the legacy 55-byte packet.

Command (laptop → ESP32, event-driven):
    Fixed 22-byte packet.
"""

import struct
from dataclasses import dataclass, field
from typing import Optional

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PKT_TELEM   = 0x01   # telemetry from drone
PKT_CMD     = 0x02   # command to drone

CMD_GOTO          = 0x01   # navigate to (goal_x, goal_y)
CMD_LAND          = 0x02   # land immediately
CMD_HOLD          = 0x03   # hold position, cancel goal
CMD_SET_NAV_TAGS  = 0x04   # send navigation tag map positions to drone
CMD_START         = 0x05   # arm and take off
CMD_SET_PEERS     = 0x06   # update nearby drone positions for inter-drone avoidance

VFH_BINS    = 32

# Nav state values (must match nav_state_t in nav_task.h)
NAV_IDLE       = 0
NAV_ROTATING   = 1
NAV_FLYING     = 2
NAV_ARRIVED    = 3
NAV_STUCK      = 4
NAV_RETREATING = 5

NAV_STATE_NAMES = {
    NAV_IDLE:       "IDLE",
    NAV_ROTATING:   "ROTATING",
    NAV_FLYING:     "FLYING",
    NAV_ARRIVED:    "ARRIVED",
    NAV_STUCK:      "STUCK",
    NAV_RETREATING: "RETREATING",
}

# ---------------------------------------------------------------------------
# Telemetry packet
# ---------------------------------------------------------------------------

# Wire format begins with the legacy 55-byte packet. New firmware appends PX4
# link health, local Z/velocity, and full attitude; appending the extensions lets
# new laptop code read old firmware and old laptop code ignore the extra bytes.
_TELEM_HDR_FMT  = "<BBfffBbf32sBH"
_TELEM_HDR_SIZE = struct.calcsize(_TELEM_HDR_FMT)   # 55 bytes
_TELEM_STATUS_FMT = "<HHHBBB"
_TELEM_STATUS_SIZE = struct.calcsize(_TELEM_STATUS_FMT)  # 9 bytes
_TELEM_STATUS_V0_FMT = "<HHBBB"
_TELEM_STATUS_V0_SIZE = struct.calcsize(_TELEM_STATUS_V0_FMT)  # 7 bytes
_TELEM_KIN_FMT = "<ffff"
_TELEM_KIN_SIZE = struct.calcsize(_TELEM_KIN_FMT)        # 16 bytes
# Matches extension 2 in wifi_telem_pkt_t (main/wifi_task.h).
_TELEM_ATT_FMT = "<9fIIB"
_TELEM_TIMING_FMT = "<QqIIHBBIBBBBI"
_TELEM_TIMING_SIZE = struct.calcsize(_TELEM_TIMING_FMT)
_TELEM_TOF_EXT_FMT = "<B"   # appended: ToF integration time (ms), 0 = ToF off
_TELEM_ATT_SIZE = struct.calcsize(_TELEM_ATT_FMT)        # 45 bytes


@dataclass
class TelemetryPacket:
    drone_id:    int
    ned_x:       float
    ned_y:       float
    heading_rad: float
    nav_state:   int
    tag_id:      int              # −1 if no tag visible
    tag_dist_m:  float
    vfh_blocked: list            # VFH_BINS bools
    is_stuck:    bool
    reloc_age_s: int              # seconds since last nav-tag fix (0xFFFF = never)
    px4_hb_age_ms: Optional[int] = None
    px4_pos_age_ms: Optional[int] = None
    px4_att_age_ms: Optional[int] = None
    px4_link_ok: Optional[bool] = None
    px4_armed: Optional[int] = None
    px4_custom_main_mode: Optional[int] = None
    ned_z: Optional[float] = None
    ned_vx: Optional[float] = None
    ned_vy: Optional[float] = None
    ned_vz: Optional[float] = None
    q_w: Optional[float] = None
    q_x: Optional[float] = None
    q_y: Optional[float] = None
    q_z: Optional[float] = None
    roll_rate: Optional[float] = None
    pitch_rate: Optional[float] = None
    yaw_rate: Optional[float] = None
    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    px4_pos_boot_ms: Optional[int] = None
    px4_att_boot_ms: Optional[int] = None
    q_from_px4: Optional[int] = None

    esp_send_us: Optional[int] = None
    clock_offset_us: Optional[int] = None
    sync_rtt_us: Optional[int] = None
    sync_age_us: Optional[int] = None
    sync_samples: Optional[int] = None
    sync_locked: Optional[int] = None
    experiment_phase: Optional[int] = None
    mission_id: Optional[int] = None
    corner: Optional[int] = None
    tof_side: Optional[int] = None
    tof_hz: Optional[int] = None
    passive: Optional[int] = None
    boot_id: Optional[int] = None
    tof_integration_ms: Optional[int] = None

    @property
    def nav_state_name(self) -> str:
        return NAV_STATE_NAMES.get(self.nav_state, f"UNKNOWN({self.nav_state})")


def parse_telemetry(data: bytes) -> Optional[TelemetryPacket]:
    """Parse a raw UDP payload into a TelemetryPacket. Returns None on error."""
    if len(data) < _TELEM_HDR_SIZE:
        return None

    fields = struct.unpack_from(_TELEM_HDR_FMT, data, 0)
    (pkt_type, drone_id, ned_x, ned_y, heading_rad,
     nav_state, tag_id, tag_dist_m,
     vfh_raw, is_stuck, reloc_age_s) = fields

    if pkt_type != PKT_TELEM:
        return None

    vfh_blocked = [bool(b) for b in vfh_raw]

    status = (None, None, None, None, None, None)
    status_end = _TELEM_HDR_SIZE + _TELEM_STATUS_SIZE
    if len(data) >= status_end:
        hb_age, pos_age, att_age, link_ok, armed, main_mode = struct.unpack_from(
            _TELEM_STATUS_FMT, data, _TELEM_HDR_SIZE)
        status = (hb_age, pos_age, att_age, bool(link_ok), armed, main_mode)
    elif len(data) >= _TELEM_HDR_SIZE + _TELEM_STATUS_V0_SIZE:
        hb_age, pos_age, armed, main_mode, _sub_mode = struct.unpack_from(
            _TELEM_STATUS_V0_FMT, data, _TELEM_HDR_SIZE)
        status = (hb_age, pos_age, None,
                  hb_age < 2000 and pos_age < 500, armed, main_mode)

    kinematics = (None, None, None, None)
    if len(data) >= status_end + _TELEM_KIN_SIZE:
        kinematics = struct.unpack_from(_TELEM_KIN_FMT, data, status_end)

    attitude = (None,) * 12
    attitude_start = status_end + _TELEM_KIN_SIZE
    if len(data) >= attitude_start + _TELEM_ATT_SIZE:
        attitude = struct.unpack_from(_TELEM_ATT_FMT, data, attitude_start)

    timing = (None,) * 13
    timing_start = attitude_start + _TELEM_ATT_SIZE
    if len(data) >= timing_start + _TELEM_TIMING_SIZE:
        timing = struct.unpack_from(_TELEM_TIMING_FMT, data, timing_start)

    tof_integration_ms = None
    ext_start = timing_start + _TELEM_TIMING_SIZE
    if len(data) >= ext_start + struct.calcsize(_TELEM_TOF_EXT_FMT):
        (tof_integration_ms,) = struct.unpack_from(_TELEM_TOF_EXT_FMT, data, ext_start)

    return TelemetryPacket(
        drone_id    = drone_id,
        ned_x       = ned_x,
        ned_y       = ned_y,
        heading_rad = heading_rad,
        nav_state   = nav_state,
        tag_id      = tag_id,
        tag_dist_m  = tag_dist_m,
        vfh_blocked = vfh_blocked,
        is_stuck    = bool(is_stuck),
        reloc_age_s = reloc_age_s,
        px4_hb_age_ms = status[0],
        px4_pos_age_ms = status[1],
        px4_att_age_ms = status[2],
        px4_link_ok = status[3],
        px4_armed = status[4],
        px4_custom_main_mode = status[5],
        ned_z = kinematics[0],
        ned_vx = kinematics[1],
        ned_vy = kinematics[2],
        ned_vz = kinematics[3],
        q_w = attitude[0],
        q_x = attitude[1],
        q_y = attitude[2],
        q_z = attitude[3],
        roll_rate = attitude[4],
        pitch_rate = attitude[5],
        yaw_rate = attitude[6],
        roll_rad = attitude[7],
        pitch_rad = attitude[8],
        px4_pos_boot_ms = attitude[9],
        px4_att_boot_ms = attitude[10],
        q_from_px4 = attitude[11],
        esp_send_us = timing[0],
        clock_offset_us = timing[1],
        sync_rtt_us = timing[2],
        sync_age_us = timing[3],
        sync_samples = timing[4],
        sync_locked = timing[5],
        experiment_phase = timing[6],
        mission_id = timing[7],
        corner = timing[8],
        tof_side = timing[9],
        tof_hz = timing[10],
        passive = timing[11],
        boot_id = timing[12],
        tof_integration_ms = tof_integration_ms,

    )

# ---------------------------------------------------------------------------
# Command packet
# ---------------------------------------------------------------------------

# pkt_type, cmd_type, goal_x, goal_y, found_tag_ids[12] (int8_t, −1 = unused)
_CMD_FMT  = "<BBff12b"
_CMD_SIZE = struct.calcsize(_CMD_FMT)   # 22 bytes

MAX_FOUND_TAGS = 12


@dataclass
class CommandPacket:
    cmd_type:      int
    goal_x:        float     = 0.0
    goal_y:        float     = 0.0
    found_tag_ids: list      = field(default_factory=list)  # up to 8 tag IDs


def build_command(cmd: CommandPacket) -> bytes:
    """Serialise a CommandPacket to bytes ready to send over UDP."""
    tag_ids = (cmd.found_tag_ids + [-1] * MAX_FOUND_TAGS)[:MAX_FOUND_TAGS]
    return struct.pack(_CMD_FMT, PKT_CMD, cmd.cmd_type,
                       cmd.goal_x, cmd.goal_y, *tag_ids)


# ---------------------------------------------------------------------------
# Navigation-tag position packet  (laptop → drone)
# ---------------------------------------------------------------------------

MAX_NAV_TAGS = 16

# Wire format per tag entry: int8_t id + float map_x + float map_y
_NAV_TAG_ENTRY_FMT = "<bff"
_NAV_TAG_ENTRY_SIZE = struct.calcsize(_NAV_TAG_ENTRY_FMT)  # 9 bytes


@dataclass
class NavTag:
    """A navigation AprilTag at a known map-frame position."""
    id:    int      # AprilTag ID
    map_x: float    # NED north in map frame (m)
    map_y: float    # NED east  in map frame (m)


def build_nav_tags_command(tags: list[NavTag],
                           start_x: float = 0.0,
                           start_y: float = 0.0) -> bytes:
    """
    Build a CMD_SET_NAV_TAGS packet for one specific drone.

    tags:    list of NavTag in map frame (up to MAX_NAV_TAGS).
    start_x: drone's start position in map frame (NED north, m).
    start_y: drone's start position in map frame (NED east,  m).

    Tag positions are pre-converted to the drone's odom frame
    (map_pos − start_offset) so the drone never needs to know the
    map frame directly.
    """
    count = min(len(tags), MAX_NAV_TAGS)
    # Header: pkt_type, cmd_type, count, start_map_x, start_map_y
    buf = struct.pack("<BBBff", PKT_CMD, CMD_SET_NAV_TAGS, count, start_x, start_y)
    for t in tags[:count]:
        odom_x = t.map_x - start_x
        odom_y = t.map_y - start_y
        buf += struct.pack(_NAV_TAG_ENTRY_FMT, t.id, odom_x, odom_y)
    return buf


# ---------------------------------------------------------------------------
# Peer drone positions packet  (laptop → drone)
# ---------------------------------------------------------------------------

MAX_PEERS = 7


def build_peers_command(peers: list[tuple[float, float]]) -> bytes:
    """
    Build a CMD_SET_PEERS packet.

    peers: list of (map_x, map_y) positions in map frame, up to MAX_PEERS.
    Returns bytes ready to send over UDP.
    """
    count = min(len(peers), MAX_PEERS)
    buf = struct.pack("<BBB", PKT_CMD, CMD_SET_PEERS, count)
    for mx, my in peers[:count]:
        buf += struct.pack("<ff", mx, my)
    return buf
