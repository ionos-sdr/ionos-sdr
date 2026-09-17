#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# usb_sink.py — a PC oldal az usb_bench.ino-hoz
#   Copyright (c) 2026 Zoltan Doczi HA7DCD
#
# Beolvassa az ESP32-S3 NATIV USB portjat, amilyen gyorsan csak tudja, es
# masodpercenkent kiirja a tenylegesen atjott sebesseget.
#
# MIERT KELL EZ: az USB CDC-n a gazdagep kezdemenyez minden atvitelt. Ha
# senki nem olvas, az ESP TX puffere tele lesz, es 0 kB/s-t fogsz merni —
# a linkrol semmit. Az usb_bench.ino szama CSAK ezzel egyutt ervenyes.
#
# HASZNALAT
#   python usb_sink.py COM4
#   python usb_sink.py /dev/ttyACM0
#   python usb_sink.py COM4 --count-only     (nincs keret-ellenorzes,
#                                             csak bajtszamlalas — igy latod,
#                                             hogy nem a Python a szuk hely)
#
# MIT NEZZ:
#   - a ket oldal (ESP CP2102-je es ez) kB/s-e egyezzen. Ha az ESP tobbet
#     mond, a gazdagep-oldali olvasas a szuk keresztmetszet.
#   - hianyzo = a sorszamokbol szamolt vesztes. USB CDC-n ennek 0-nak
#     KELLENE lennie (a CDC bulk transfer nyugtazott); ha nem az, akkor az
#     ESP dobta el kuldes elott, nem a vezetek vesztette el.

import argparse
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("Hianyzik a pyserial:  pip install pyserial")

MAGIC = b"IQB2"
HDR = 16
BLK = HDR + 256 * 4          # 1040


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("device", help="a NATIV USB port, pl. COM4 / /dev/ttyACM0")
    ap.add_argument("--count-only", action="store_true",
                    help="csak bajtokat szamol, nem parsol keretet")
    ap.add_argument("--baud", type=int, default=921600,
                    help="nativ USB CDC-nel ertelmetlen, de nem art")
    args = ap.parse_args()

    ser = serial.Serial(args.device, args.baud, timeout=0.05)
    try:
        ser.set_buffer_size(rx_size=1 << 20)   # csak Windowson letezik
    except AttributeError:
        pass

    print(f"olvasas: {args.device}")
    print("mod:", "csak bajtszamlalas" if args.count_only else "keret-ellenorzes")
    print("varakozas az elso bajtra...\n")

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
                print(f"{kbs:8.1f} kB/s   ({sps:7d} sps 16 biten)")
            else:
                print(f"{kbs:8.1f} kB/s   {win_blocks:6d} blokk/s   "
                      f"hianyzo={win_lost:<6d} ujraszink={win_resync:<4d} "
                      f"({sps:7d} sps 16 biten)")
            win_bytes = win_blocks = win_lost = win_resync = 0
            t_win = now

        if t_all and (now - t_all) > 1.0 and not chunk:
            # 1 masodpercnel regebbi az utolso bajt -> vege a meresnek
            if now - t_win > 3.0:
                break

    print("\n=========== OSSZEFOGLALO ===========")
    print(f"osszesen atjott: {total_bytes/1024.0/1024.0:.2f} MiB")
    print(f"csucs:           {peak_kbs:.1f} kB/s")
    print(f"ez 16 bites I/Q-val: {int(peak_kbs*1024/4)} sps")
    print(f"       8 bitesnel:   {int(peak_kbs*1024/2)} sps")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nvege")
