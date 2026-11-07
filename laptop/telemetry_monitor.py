#!/usr/bin/env python3
"""Passively display/log custom drone telemetry, leaving QGC's UDP 14550 free.

    python laptop/telemetry_monitor.py --drone-id 22
    python laptop/telemetry_monitor.py --duration 30 --output telemetry.csv

Receives UDP 5005 only; does not send commands. Stop other receivers on 5005
before running. Missing fields from older firmware are written as empty cells.
The firmware sends latest-state snapshots at 10 Hz, not every MAVLink sample.
ned_x/ned_y include the configured map offset; ned_z is PX4 local NED down.
q_from_px4=1 means received quaternion, 0 means Euler-derived fallback.
Always check px4_att_age_ms and px4_att_boot_ms for freshness as well.
"""

import argparse
import csv
from dataclasses import asdict, fields
from pathlib import Path
import socket
import sys
import time
from contextlib import nullcontext

from protocol import TelemetryPacket, parse_telemetry


CSV_FIELDS = ["host_receive_ns", "source_ip"] + [
    f.name for f in fields(TelemetryPacket) if f.name != "vfh_blocked"
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--drone-id", type=int)
    parser.add_argument("--duration", type=float, help="Seconds to receive; default until Ctrl+C")
    parser.add_argument("--output", type=Path, help="New CSV file; default stdout")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")

    count = 0
    try:
        # Exclusive bind so another telemetry receiver cannot silently steal packets.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(("0.0.0.0", args.port))
            sock.settimeout(0.5)
            output = args.output.open("x", newline="") if args.output else nullcontext(sys.stdout)
            with output as handle:
                writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
                writer.writeheader()
                handle.flush()
                print(f"Listening on UDP {args.port}; QGC can use UDP 14550.", file=sys.stderr)
                deadline = time.monotonic() + args.duration if args.duration else None
                try:
                    while deadline is None or time.monotonic() < deadline:
                        try:
                            data, source = sock.recvfrom(65535)
                        except socket.timeout:
                            continue
                        received_ns = time.time_ns()
                        packet = parse_telemetry(data)
                        if packet is None or (args.drone_id is not None and packet.drone_id != args.drone_id):
                            continue
                        writer.writerow(dict(asdict(packet), host_receive_ns=received_ns, source_ip=source[0]))
                        handle.flush()
                        count += 1
                except KeyboardInterrupt:
                    pass
    except OSError as exc:
        print(f"Cannot receive/log telemetry: {exc}", file=sys.stderr)
        return 1
    print(f"Received {count} telemetry packets.", file=sys.stderr)
    return 0 if count else 1


if __name__ == "__main__":
    raise SystemExit(main())
