#!/usr/bin/env python3
"""Live 8-sensor ToF heatmaps: red = near, blue = far, grey = unavailable.

    python3 laptop/tof_heatmap.py
    python3 laptop/tof_heatmap.py --source 192.168.1.42 --front-idx 1
    python3 laptop/tof_heatmap.py --demo

Receives only; never sends commands. Requires matplotlib and numpy.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import socket
import struct
import time


SENSORS = 8
PIXELS = 64
VALID_STATUS = {5, 9}  # Same accepted target statuses as tof_task.c.
PACKET = struct.Struct("<BBBI64h64B")  # Signed raw distances; 199-byte prefix.
UPTIME = struct.Struct("<I")
UINT32_MASK = 0xFFFFFFFF


@dataclass(frozen=True)
class Frame:
    sensor: int
    ok: bool
    timestamp_ms: int
    distances: tuple[int, ...]
    statuses: tuple[int, ...]
    sent_ms: int | None = None


def decode_packet(data: bytes) -> Frame | None:
    """Accept the legacy prefix and the extended packet; reject other traffic."""
    if len(data) not in (PACKET.size, PACKET.size + UPTIME.size):
        return None
    fields = PACKET.unpack_from(data)
    if fields[0] != 0x03 or fields[1] >= SENSORS or fields[2] not in (0, 1):
        return None
    sent_ms = UPTIME.unpack_from(data, PACKET.size)[0] if len(data) > PACKET.size else None
    if sent_ms is not None and ((sent_ms - fields[3]) & UINT32_MASK) >= 0x80000000:
        return None  # A frame cannot be newer than the snapshot that contains it.
    return Frame(fields[1], bool(fields[2]), fields[3],
                 fields[4:68], fields[68:132], sent_ms)


@dataclass
class Reading:
    frame: Frame
    observed_at: float  # Host monotonic time of acquisition, estimated at receipt.

    def age_ms(self, now: float) -> float:
        return max(0.0, (now - self.observed_at) * 1000.0)

    def state(self, now: float, fresh_ms: float, recent_ms: float) -> str:
        if not self.frame.ok:
            return "OFFLINE"
        age = self.age_ms(now)
        if age <= fresh_ms:
            return "FRESH"
        return "RECENT" if age <= recent_ms else "STALE"


class Ring:
    """Keep independent sensor ages; duplicate transmissions do not refresh them."""

    def __init__(self):
        self.readings: list[Reading | None] = [None] * SENSORS
        self.last_sent_ms: int | None = None

    def update(self, frame: Frame, now: float) -> None:
        # A large uptime drop signals a reboot. Small reordered packets are ignored
        # below; natural uint32 wrap still has a small positive modular delta.
        if frame.sent_ms is not None:
            if self.last_sent_ms is not None:
                delta = (frame.sent_ms - self.last_sent_ms) & UINT32_MASK
                if delta >= 0x80000000:
                    if self.last_sent_ms - frame.sent_ms > 1000:
                        self.readings = [None] * SENSORS
                    else:
                        return
            self.last_sent_ms = frame.sent_ms

        previous = self.readings[frame.sensor]
        source_age = ((frame.sent_ms - frame.timestamp_ms) & UINT32_MASK) / 1000.0 \
            if frame.sent_ms is not None else 0.0
        observed_at = now - source_age
        if previous is not None:
            delta = (frame.timestamp_ms - previous.frame.timestamp_ms) & UINT32_MASK
            if delta == 0:
                # Keep offline/online state current without making a frozen frame fresh.
                observed_at = min(observed_at, previous.observed_at)
            elif delta >= 0x80000000:
                return
        self.readings[frame.sensor] = Reading(frame, observed_at)


def valid_pixels(frame: Frame, minimum: int, maximum: int) -> list[bool]:
    return [status in VALID_STATUS and minimum <= distance <= maximum
            for distance, status in zip(frame.distances, frame.statuses)]


def front_index() -> int:
    """Use this checkout's menuconfig value without reading out Wi-Fi credentials."""
    config = Path(__file__).resolve().parents[1] / "sdkconfig"
    if config.exists():
        for line in config.read_text().splitlines():
            if line.startswith("CONFIG_TOF_FRONT_SENSOR_IDX="):
                return int(line.split("=", 1)[1])
    return 0  # Kconfig default.


def demo_frames(elapsed: float) -> list[Frame]:
    """Synthetic moving depths, invalid cells, and one deliberately frozen sensor."""
    frames = []
    for sensor in range(SENSORS):
        acquired = min(elapsed, 1.0) if sensor == 7 else max(0.0, elapsed - 0.65) if sensor == 6 else elapsed
        distances = tuple(int(200 + 3600 * (0.5 + 0.5 * math.sin(
            acquired + sensor * 0.6 + row * 0.25 + col * 0.3)))
            for row in range(8) for col in range(8))
        statuses = tuple(255 if i % 19 == 0 else 9 if i % 13 == 0 else 5
                         for i in range(PIXELS))
        frames.append(Frame(sensor, True, int(acquired * 1000), distances,
                            statuses, int(elapsed * 1000)))
    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bind", default="0.0.0.0", help="UDP bind address")
    parser.add_argument("--port", type=int, default=5007)
    parser.add_argument("--source", help="ESP32 IPv4 address; otherwise lock to first sender")
    parser.add_argument("--front-idx", type=int, choices=range(8), default=front_index())
    parser.add_argument("--fresh-ms", type=float, default=250)
    parser.add_argument("--recent-ms", type=float, default=1500,
                        help="Hide readings older than this (default: 1500 ms)")
    parser.add_argument("--min-mm", type=int, default=50)
    parser.add_argument("--max-mm", type=int, default=4000)
    parser.add_argument("--demo", action="store_true", help="Show clearly labelled synthetic data")
    parser.add_argument("--save", type=Path, help="With --demo, save a preview PNG and exit")
    args = parser.parse_args()
    if not (0 < args.fresh_ms < args.recent_ms) or not (0 < args.min_mm < args.max_mm):
        parser.error("Require 0 < fresh-ms < recent-ms and 0 < min-mm < max-mm")
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    if args.save and not args.demo:
        parser.error("--save requires --demo")

    try:
        import matplotlib
        if args.save:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.colors import LinearSegmentedColormap, Normalize
        import numpy as np
    except ImportError:
        parser.exit(1, "Install display dependencies: python3 -m pip install matplotlib numpy\n")

    sock = None
    if not args.demo:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((args.bind, args.port))
        except OSError as exc:
            sock.close()
            parser.exit(1, f"Cannot listen on UDP {args.bind}:{args.port}: {exc}\n"
                        "Close tof_debug.py or another viewer using the same port.\n")
        sock.setblocking(False)
        print(f"Listening on UDP {args.bind}:{args.port}. Waiting for ESP32 ToF frames.")

    ring = Ring()
    source = args.source
    started = time.monotonic()
    cmap = LinearSegmentedColormap.from_list("near_to_far", ["#ef3535", "#2864ed"])
    cmap.set_bad("#d1d5db")
    norm = Normalize(args.min_mm / 1000, args.max_mm / 1000, clip=True)
    fig, axes = plt.subplots(2, 4, figsize=(16, 9))
    fig.subplots_adjust(left=0.04, right=0.92, bottom=0.14, top=0.83,
                        hspace=0.42, wspace=0.25)
    images, labels = [], []
    for sensor, ax in enumerate(axes.flat):
        image = ax.imshow(np.full((8, 8), np.nan), cmap=cmap, norm=norm,
                          origin="upper", interpolation="nearest")
        images.append(image)
        labels.append([ax.text(col, row, "", ha="center", va="center", fontsize=8)
                       for row in range(8) for col in range(8)])
        ax.set_xticks(range(8))
        ax.set_yticks(range(8))
        ax.tick_params(labelsize=8)
        ax.set_xlabel("raw column", fontsize=9)
        ax.set_ylabel("raw row", fontsize=9)

        def cursor(x, y, index=sensor):
            col, row = int(x + 0.5), int(y + 0.5)
            reading = ring.readings[index]
            if reading is None or not (-0.5 <= x < 7.5 and -0.5 <= y < 7.5):
                return "No pixel"
            pixel = row * 8 + col
            return (f"S{index} [{row},{col}]: {reading.frame.distances[pixel]} mm; "
                    f"status={reading.frame.statuses[pixel]}; "
                    f"age={reading.age_ms(time.monotonic()):.0f} ms")
        ax.format_coord = cursor

    color_ax = fig.add_axes([0.94, 0.2, 0.015, 0.5])
    bar = fig.colorbar(images[0], cax=color_ax)
    bar.ax.invert_yaxis()  # Near red above far blue.
    bar.set_label("Distance (m) — red near / blue far")
    heading = fig.suptitle("", fontsize=17, y=0.97)
    connection = fig.text(0.5, 0.91, "", ha="center", fontsize=11)
    fig.text(0.5, 0.025,
             f"Fresh ≤ {args.fresh_ms:g} ms  |  Recent ≤ {args.recent_ms:g} ms  |  "
             "Stale / offline / invalid = grey  |  Cell values in metres; * = weak signal\n"
             "Raw sensor row/column order; mounting rotation and lens mirroring are not corrected.",
             ha="center", fontsize=10)

    def update(_):
        nonlocal source
        now = time.monotonic()
        if args.demo:
            for frame in demo_frames(3.0 if args.save else now - started):
                ring.update(frame, now)
        else:
            # Bound socket work so a noisy sender cannot freeze the UI or stale timers.
            for _ in range(512):
                try:
                    data, address = sock.recvfrom(2048)
                except BlockingIOError:
                    break
                if source is not None and address[0] != source:
                    continue
                frame = decode_packet(data)
                if frame is None:
                    continue
                if source is None:
                    source = address[0]
                    print(f"Using ESP32 sender {source}; other senders will be ignored.")
                ring.update(frame, now)

        fresh_count = 0
        for sensor, ax in enumerate(axes.flat):
            reading = ring.readings[sensor]
            bearing = ((args.front_idx - sensor) * 45) % 360
            location = "FRONT" if bearing == 0 else f"{bearing}° CW"
            state, detail = "WAITING", "no frame received"
            values = np.full((8, 8), np.nan)
            mask = [False] * PIXELS
            if reading is not None:
                state = reading.state(now, args.fresh_ms, args.recent_ms)
                mask = valid_pixels(reading.frame, args.min_mm, args.max_mm)
                detail = f"age {reading.age_ms(now):.0f} ms · {sum(mask)}/64 valid"
                if state in ("FRESH", "RECENT"):
                    values = np.where(np.array(mask).reshape(8, 8),
                                      np.array(reading.frame.distances).reshape(8, 8) / 1000,
                                      np.nan)
                fresh_count += state == "FRESH"
            images[sensor].set_data(np.ma.masked_invalid(values))
            images[sensor].set_alpha(0.65 if state == "RECENT" else 1.0)
            color = "#087f5b" if state == "FRESH" else "#a16207" if state == "RECENT" else "#6b7280"
            ax.set_title(f"S{sensor} / mux {sensor} · {location}\n{state} · {detail}",
                         fontsize=10, color=color)
            for pixel, label in enumerate(labels[sensor]):
                distance = values.flat[pixel]
                weak = reading is not None and reading.frame.statuses[pixel] == 9
                label.set_text(f"{distance:.2f}{'*' if weak else ''}" if math.isfinite(distance) else "—")
                label.set_color("white" if math.isfinite(distance) else "#6b7280")
        heading.set_text(f"{'DEMO — synthetic data | ' if args.demo else ''}"
                         f"ToF ring · {fresh_count}/8 sensors fresh")
        connection.set_text("Synthetic targets; S6 is delayed 650 ms; S7 stops updating after 1 second."
                            if args.demo else f"UDP :{args.port} · sender {source or 'waiting'} · "
                            "all 8 grids require the updated NUS firmware")

    try:
        update(0)
        if args.save:
            fig.savefig(args.save, dpi=130)
            print(f"Saved synthetic preview: {args.save}")
        else:
            animation = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
            plt.show()
            del animation
    except KeyboardInterrupt:
        pass
    finally:
        if sock is not None:
            sock.close()
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
