#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# usb_sink.py — the PC side for usb_bench.ino
#   Copyright (c) 2026 Zoltan Doczi HA7DCD
#
# Reads the ESP32-S3 NATIVE USB port as fast as it can and prints the
# actually received rate every second.
#
# WHY THIS IS NEEDED: on USB CDC the host initiates every transfer. If
# nobody reads, the ESP TX buffer fills up and 0 kB/s is measured — nothing
# about the link. The usb_bench.ino figure is valid ONLY together with this.
#
# USAGE
#   python usb_sink.py COM4
#   python usb_sink.py /dev/ttyACM0
#   python usb_sink.py COM4 --count-only     (no frame check, byte counting
#                                             only — shows that Python is not
#                                             the bottleneck)
#
# WHAT TO LOOK AT:
#   - the kB/s of both sides (the ESP CP2102 and this) must match. If the
#     ESP reports more, the host-side read is the bottleneck.
#   - missing = loss computed from the sequence numbers. On USB CDC this
#     SHOULD be 0 (CDC bulk transfer is acknowledged); if it is not, the ESP
#     dropped it before sending, the cable did not lose it.

import argparse
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial is missing:  pip install pyserial")

MAGIC = b"IQB2"
HDR = 16
BLK = HDR + 256 * 4          # 1040


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("device", help="the NATIVE USB port, e.g. COM4 / /dev/ttyACM0")
    ap.add_argument("--count-only", action="store_true",
                    help="count bytes only, do not parse frames")
    ap.add_argument("--baud", type=int, default=921600,
                    help="meaningless for native USB CDC, but harmless")
    args = ap.parse_args()

    ser = serial.Serial(args.device, args.baud, timeout=0.05)
    try:
        ser.set_buffer_size(rx_size=1 << 20)   # exists only on Windows
    except AttributeError:
        pass

    print(f"reading: {args.device}")
    print("mode:", "byte counting only" if args.count_only else "frame check")
    print("waiting for the first byte...\n")

    buf = bytearray()
    last_seq = None
    synced = False

    total_bytes = 0
    win_bytes = 0
    win_blocks = 0
    win_lost = 0
    win_resync = 0
    peak_kbs = 0.0
    t_win = time.perf_counter()
    t_all = None

    while True:
        n = ser.in_waiting
        chunk = ser.read(n if n else 1)
        if chunk:
            if t_all is None:
                t_all = time.perf_counter()
            total_bytes += len(chunk)
            win_bytes += len(chunk)

            if not args.count_only:
                buf.extend(chunk)
                while True:
                    if not synced:
                        i = buf.find(MAGIC)
                        if i < 0:
                            if len(buf) > 4 * BLK:
                                del buf[:-3]
                            break
                        if i:
                            del buf[:i]
                            win_resync += 1
                        synced = True
                    if len(buf) < BLK:
                        break
                    if buf[:4] != MAGIC:
                        synced = False
                        continue
                    seq = int.from_bytes(buf[4:8], "little")
                    del buf[:BLK]
                    if last_seq is not None:
                        d = (seq - last_seq) & 0xFFFFFFFF
                        if 1 < d < 1_000_000:
                            win_lost += d - 1
                    last_seq = seq
                    win_blocks += 1

        now = time.perf_counter()
        dt = now - t_win
        if dt >= 1.0:
            kbs = win_bytes / 1024.0 / dt
            if kbs > peak_kbs:
                peak_kbs = kbs
            sps = int(win_bytes / dt / 4)
            if args.count_only:
                print(f"{kbs:8.1f} kB/s   ({sps:7d} sps at 16 bits)")
            else:
                print(f"{kbs:8.1f} kB/s   {win_blocks:6d} blocks/s   "
                      f"missing={win_lost:<6d} resync={win_resync:<4d} "
                      f"({sps:7d} sps at 16 bits)")
            win_bytes = win_blocks = win_lost = win_resync = 0
            t_win = now

        if t_all and (now - t_all) > 1.0 and not chunk:
            # last byte older than 1 second -> end of measurement
            if now - t_win > 3.0:
                break

    print("\n=========== SUMMARY ===========")
    print(f"total received:  {total_bytes/1024.0/1024.0:.2f} MiB")
    print(f"peak:            {peak_kbs:.1f} kB/s")
    print(f"with 16-bit I/Q:     {int(peak_kbs*1024/4)} sps")
    print(f"with 8-bit I/Q:      {int(peak_kbs*1024/2)} sps")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\ndone")
