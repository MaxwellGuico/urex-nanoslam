import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from protocol import parse_telemetry


def test_parse_telemetry_includes_px4_status_fields() -> None:
    raw = struct.pack(
        "<BBfffBbf32sBHHHBBB",
        0x01,
        2,
        1.0,
        2.0,
        0.3,
        0,
        -1,
        0.5,
        b"\x00" * 32,
        0,
        0,
        0,
        0,
        1,
        6,
        0,
    )

    pkt = parse_telemetry(raw)
    assert pkt is not None
    assert pkt.px4_link_ok is True
    assert pkt.px4_hb_age_ms == 0
    assert pkt.px4_armed == 1
    assert pkt.px4_custom_main_mode == 6


@pytest.mark.parametrize("source", [0, 1])
def test_full_firmware_telemetry_attitude_extension(source):
    # Packed C layout: legacy 55 + health 9 + kinematics 16 + attitude 45.
    prefix = struct.pack(
        "<BBfffBbf32sBHHHHBBB4f", 1, 22, 1., 2., .3, 0, -1, .5,
        bytes(32), 0, 65535, 10, 20, 30, 1, 1, 6, -1., .1, .2, .3,
    )
    raw = prefix + struct.pack(
        "<9fIIB", .5, -.5, .5, -.5, .1, -.2, .3, -.4, .6,
        123456, 123457, source,
    )
    assert len(raw) == 125
    pkt = parse_telemetry(raw)
    assert pkt is not None
    assert (pkt.q_w, pkt.q_x, pkt.q_y, pkt.q_z) == (.5, -.5, .5, -.5)
    assert (pkt.roll_rate, pkt.pitch_rate, pkt.yaw_rate) == pytest.approx((.1, -.2, .3))
    assert (pkt.roll_rad, pkt.pitch_rad) == pytest.approx((-.4, .6))
    assert pkt.px4_pos_boot_ms == 123456
    assert pkt.px4_att_boot_ms == 123457
    assert pkt.q_from_px4 == source
    # Older firmware and an incomplete extension must not fabricate attitude.
    for length in (55, 64, 80, 124):
        older = parse_telemetry(raw[:length])
        assert older is not None
        assert older.q_w is None
        assert older.q_from_px4 is None
