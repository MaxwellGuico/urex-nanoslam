"""Protocol and freshness checks for the live ring viewer, without a GUI."""

import socket
import unittest

from tof_heatmap import Frame, PACKET, Ring, UPTIME, decode_packet, valid_pixels


def packet(sensor=0, timestamp=1000, sent=1100, distance=800, status=5, ok=1):
    data = PACKET.pack(3, sensor, ok, timestamp, *([distance] * 64), *([status] * 64))
    return data if sent is None else data + UPTIME.pack(sent)


class HeatmapTests(unittest.TestCase):
    def test_wire_layout_matches_packed_c_packet(self):
        self.assertEqual(PACKET.size, 199)
        self.assertEqual(len(packet()), 203)
        frame = decode_packet(packet(sensor=7, distance=-15, status=255))
        self.assertEqual((frame.sensor, frame.timestamp_ms, frame.sent_ms), (7, 1000, 1100))
        self.assertEqual(frame.distances, (-15,) * 64)
        self.assertEqual(frame.statuses, (255,) * 64)

    def test_legacy_packets_remain_supported(self):
        frame = decode_packet(packet(sent=None))
        self.assertIsNone(frame.sent_ms)
        self.assertTrue(frame.ok)

    def test_malformed_and_unrelated_traffic_is_ignored(self):
        for data in (b"", packet()[:-1], packet() + b"x", b"\x01" + packet()[1:],
                     packet(sensor=8), packet(ok=2), packet(timestamp=1200, sent=1100)):
            with self.subTest(data=data[:8]):
                self.assertIsNone(decode_packet(data))

    def test_frozen_frame_becomes_stale_despite_continuing_packets(self):
        ring = Ring()
        ring.update(decode_packet(packet()), 10.0)
        reading = ring.readings[0]
        self.assertAlmostEqual(reading.age_ms(10.0), 100)
        self.assertEqual(reading.state(10.0, 250, 1500), "FRESH")
        ring.update(decode_packet(packet(sent=1600)), 10.5)
        self.assertEqual(ring.readings[0].state(10.5, 250, 1500), "RECENT")
        ring.update(decode_packet(packet(sent=3100)), 12.0)
        self.assertEqual(ring.readings[0].state(12.0, 250, 1500), "STALE")

    def test_first_packet_can_already_be_stale(self):
        ring = Ring()
        ring.update(decode_packet(packet(sent=5000)), 10.0)
        self.assertEqual(ring.readings[0].state(10.0, 250, 1500), "STALE")

    def test_sensors_age_independently_and_offline_overrides_freshness(self):
        ring = Ring()
        ring.update(decode_packet(packet()), 10.0)
        ring.update(decode_packet(packet(sensor=1, timestamp=3000, sent=3100)), 12.0)
        self.assertEqual(ring.readings[0].state(12.0, 250, 1500), "STALE")
        self.assertEqual(ring.readings[1].state(12.0, 250, 1500), "FRESH")
        ring.update(decode_packet(packet(sensor=1, timestamp=3000, sent=3100, ok=0)), 12.0)
        self.assertEqual(ring.readings[1].state(12.0, 250, 1500), "OFFLINE")
        self.assertIsNone(ring.readings[2])

    def test_legacy_duplicates_do_not_refresh_reading(self):
        ring = Ring()
        ring.update(decode_packet(packet(sent=None)), 10.0)
        ring.update(decode_packet(packet(sent=None)), 12.0)
        self.assertEqual(ring.readings[0].state(12.0, 250, 1500), "STALE")

    def test_uptime_wrap_does_not_reset_ring(self):
        ring = Ring()
        ring.update(decode_packet(packet(timestamp=0xFFFFFF00, sent=0xFFFFFF80)), 10.0)
        ring.update(decode_packet(packet(sensor=1, timestamp=0xFFFFFFF0, sent=32)), 10.1)
        self.assertIsNotNone(ring.readings[0])
        self.assertAlmostEqual(ring.readings[1].age_ms(10.1), 48)
        ring.update(decode_packet(packet(timestamp=16, sent=32)), 10.1)
        self.assertEqual(ring.readings[0].frame.timestamp_ms, 16)

    def test_reboot_clears_old_sensor_readings(self):
        ring = Ring()
        ring.update(decode_packet(packet(timestamp=10000, sent=10100)), 10.0)
        ring.update(decode_packet(packet(sensor=1, timestamp=50, sent=100)), 11.0)
        self.assertIsNone(ring.readings[0])
        self.assertEqual(ring.readings[1].frame.timestamp_ms, 50)

    def test_reordered_packet_does_not_replace_newer_frame(self):
        ring = Ring()
        ring.update(decode_packet(packet(timestamp=1100, sent=1200)), 10.0)
        ring.update(decode_packet(packet(timestamp=1000, sent=1100)), 10.1)
        self.assertEqual(ring.readings[0].frame.timestamp_ms, 1100)

    def test_invalid_status_and_range_never_become_far_blue_pixels(self):
        frame = Frame(0, True, 1, (49, 50, 4000, 4001, -1, 100, 100, 100),
                      (5, 5, 9, 5, 5, 255, 0, 9))
        self.assertEqual(valid_pixels(frame, 50, 4000),
                         [False, True, True, False, False, False, False, True])

    def test_udp_loopback_delivers_all_eight_frames(self):
        ring = Ring()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1)
            for sensor in range(8):
                sender.sendto(packet(sensor=sensor), receiver.getsockname())
            for _ in range(8):
                ring.update(decode_packet(receiver.recv(2048)), 10.0)
        self.assertTrue(all(reading is not None for reading in ring.readings))
        self.assertEqual([r.frame.sensor for r in ring.readings], list(range(8)))


if __name__ == "__main__":
    unittest.main()
