#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# usb_host_test.py - exercise the USB host protocol (docs/ionos-usb-protocol.md)
#   Copyright (c) 2026 Zoltan Doczi HA7DCD
#
# Opens the native USB CDC port, sends a few commands and prints every MSG1
# block (hello / response / status / error) while counting the IQB2 stream
# and its sequence gaps - the reference client for the websdr module and
# SoapyIonos.
#
#   python tools\usb_host_test.py COM7
#   python tools\usb_host_test.py COM7 --cmd "VER?" --cmd "R 100000" --cmd "FH 433775000" --sec 15
#   python tools\usb_host_test.py COM7 --raw            (no commands: v1 behaviour, hello only)

import argparse, struct, sys, time
try:
    import serial
except ImportError:
    sys.exit("pyserial is missing:  pip install pyserial")

BLK, HDR = 1040, 16
MAGICS = {b"IQB2": "IQ", b"SPC1": "SPC", b"MSG1": "MSG"}
MSG_TYPES = {0: "RESPONSE", 1: "HELLO", 2: "STATUS", 3: "ERROR"}

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--cmd", action="append", default=[], help="line to send (repeatable), sent 1 s apart")
ap.add_argument("--sec", type=float, default=10.0)
ap.add_argument("--raw", action="store_true", help="send nothing")
a = ap.parse_args()

# ESP32-S3 native USB (USB-Serial/JTAG): the chip RESETS when the host asserts RTS
# while DTR is low - pyserial's default open/close does exactly that (rst:0x15, the
# port goes silent for the ~6 s boot). The S3 has no register to turn this off
# (the P4 has USB_UART_CHIP_RST_DIS), so the host must never assert RTS: set the
# lines BEFORE open, they then go into the same DCB.
def open_port(port, baud, timeout):
    s = serial.Serial(); s.port = port; s.baudrate = baud; s.timeout = timeout
    s.rts = False; s.dtr = True
    s.open(); return s
s = open_port(a.port, 115200, 0.05)   # DTR on: the ESP sees 'connected' once we drain
buf = bytearray()
t0 = time.time()
stats = {"IQ": 0, "SPC": 0, "MSG": 0, "resync": 0, "gaps": 0}
last_seq = None
cmds = [] if a.raw else (a.cmd or ["CAP?", "VER?"])
next_cmd_t = t0 + 1.0

def handle(blk):
    global last_seq
    magic = blk[:4]
    kind = MAGICS[magic]
    seq, nsamp, decim, fs = struct.unpack_from("<IHHI", blk, 4)
    stats[kind] += 1
    if kind == "IQ":
        if last_seq is not None and seq != (last_seq + 1) & 0xFFFFFFFF:
            stats["gaps"] += 1
        last_seq = seq
    elif kind == "MSG":
        text = blk[HDR:HDR + nsamp].decode("utf-8", "replace").rstrip("\n")
        print(f"[{time.time()-t0:6.2f}s] MSG1 {MSG_TYPES.get(decim, decim)} seq={seq}:")
        for ln in text.split("\n"):
            print("    " + ln)

while time.time() - t0 < a.sec:
    if cmds and time.time() >= next_cmd_t:
        c = cmds.pop(0)
        print(f"[{time.time()-t0:6.2f}s] -> {c}")
        s.write((c + "\n").encode())
        next_cmd_t = time.time() + 1.0
    d = s.read(65536)
    if d:
        buf += d
    # Parse by offset: "buf = buf[BLK:]" copied the whole buffer for every
    # block, which at 200 ksps (781 blocks/s) made this script the bottleneck
    # (measured 2026-11-04: ~770 blocks/s and growing loss, while a reader
    # without the copies took all 781 blocks/s).
    off = 0
    while len(buf) - off >= BLK:
        if bytes(buf[off:off + 4]) in MAGICS:
            handle(bytes(buf[off:off + BLK])); off += BLK
        else:
            # resync: find the next magic
            i = -1
            for m in MAGICS:
                j = buf.find(m, off + 4)
                if j >= 0 and (i < 0 or j < i):
                    i = j
            if i < 0:
                off = len(buf) - 3; break
            off = i; stats["resync"] += 1
    del buf[:off]

dt = time.time() - t0
print(f"\n{dt:.1f} s: IQB2 {stats['IQ']} blocks ({stats['IQ']/dt:.1f}/s, {stats['IQ']*256/dt:.0f} sps), "
      f"gaps {stats['gaps']}, SPC1 {stats['SPC']}, MSG1 {stats['MSG']}, resync {stats['resync']}")
s.close()
