#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# ionos_mock.py - Ionos SDR mini USB protocol mock on a Linux pty
#   Copyright (c) 2026 Zoltan Doczi HA7DCD
#
# Speaks docs/ionos-usb-protocol.md the way firmware-esp32/streaming/src/usb_host.cpp
# does today: hello on connect and on CAP?, VER?, MODE, STAT, FH, R (with the
# firmware's R rounding), G (-> ERR), forwarding/rejection of the FG23 letters,
# periodic MSG1 status, and an IQB2 stream at fs_in/R carrying a complex test
# tone whose phase is a function of the block seq (so a host can check that its
# zero-fill kept the timing exact).
#
#   python3 ionos_mock.py --link /tmp/ionos0
#   python3 ionos_mock.py --link /tmp/ionos0 --drop-every 500 --drop-burst 2 --cut-every 777
#
# Loss injection (by seq, deterministic, so a test can recompute it):
#   --drop-every N --drop-burst K : blocks with seq % N in [N//2, N//2+K) are not sent
#   --cut-every M                 : blocks with seq % M == 7 lose 128 bytes mid-block
#                                   (two 64-byte USB packets, as seen on the real link)
# "Connected" (the DTR edge on the real device) is taken from the pty: the
# master sees POLLHUP while no process has the slave open. While nobody is
# connected, blocks are dropped (usb_drop), as the ESP does after USB_GONE_MS.

import argparse, errno, json, math, os, select, signal, struct, sys, time, tty

try:
    import numpy as np
except ImportError:
    sys.exit("numpy is missing: pip install numpy")

BLK, HDR = 1040, 16
MAGIC_IQB2, MAGIC_SPC1, MAGIC_MSG1 = 0x32425149, 0x31435053, 0x3147534D
MSG_RESPONSE, MSG_HELLO, MSG_STATUS, MSG_ERROR = 0, 1, 2, 3
FS_IN = 400000
TICK_HZ = 4.6492
FORWARD = "FofiWTts"
REJECT = "cwpxbzydkngvM"
OUT_LIMIT = 64 * 1024  # mock "TX ring": whole blocks are dropped above this (usb_drop)

ap = argparse.ArgumentParser()
ap.add_argument("--link", default="/tmp/ionos0", help="symlink to the pty slave")
ap.add_argument("--tone", type=float, default=13370.0, help="test tone offset [Hz]")
ap.add_argument("--amp", type=float, default=8000.0, help="test tone amplitude [int16]")
ap.add_argument("--rssi", type=int, default=-97)
ap.add_argument("--decim", type=int, default=8, help="autostart R (FG23 IQ_AUTOSTART)")
ap.add_argument("--drop-every", type=int, default=0)
ap.add_argument("--drop-burst", type=int, default=1)
ap.add_argument("--cut-every", type=int, default=0)
ap.add_argument("--spc-every", type=int, default=0, help="inject an SPC1 block every N IQ blocks")
ap.add_argument("--err-every", type=float, default=0.0, help="MSG1 type 3 every S seconds")
ap.add_argument("--v1", action="store_true", help="v1 firmware: no hello, no host commands")
ap.add_argument("--gain-shift-max", type=int, default=0, help="hello gain_shift_max (firmware: 0)")
ap.add_argument("--stats", default="", help="write counters as JSON here on exit")
ap.add_argument("--quiet", action="store_true")
a = ap.parse_args()


def log(*x):
    if not a.quiet:
        print(f"[mock {time.monotonic() - T0:8.3f}]", *x, flush=True)


T0 = time.monotonic()
master, slave = os.openpty()
tty.setraw(slave)
os.set_blocking(master, False)
slave_name = os.ttyname(slave)
os.close(slave)  # the host's open/close is then visible as POLLHUP on the master
poller = select.poll()
poller.register(master, select.POLLIN)
try:
    os.unlink(a.link)
except FileNotFoundError:
    pass
os.symlink(slave_name, a.link)
log(f"pty {slave_name} -> {a.link}")

st = dict(produced=0, dropped=0, cut=0, usb_drop=0, hellos=0, commands=0, spc=0, msgs=0, restarts=0)
out = bytearray()
decim = a.decim
seq = 0
epoch_t = time.monotonic()
epoch_blocks = 0
streaming = True
restart_at = None
restart_decim = None
freq_hz = 433775000
usb_mode = False
stat_period = 2.0
host_spoke = False
connected = False
last_stat = 0.0
last_err = time.monotonic()
scan = None
rx_line = bytearray()
blk_window = []


def header(magic, s, nsamp, d, fs):
    return struct.pack("<IIHHI", magic, s & 0xFFFFFFFF, nsamp, d, fs & 0xFFFFFFFF)


def queue_msg(mtype, text):
    global out
    b = text.encode()[: BLK - HDR]
    blk = header(MAGIC_MSG1, st["msgs"], len(b), mtype, 0) + b + bytes(BLK - HDR - len(b))
    st["msgs"] += 1
    out += blk  # MSG1 always fits in the mock (the ESP keeps a 4-block queue)


def hello():
    st["hellos"] += 1
    queue_msg(MSG_HELLO,
              "ionos=1\nproduct=ionos-sdr-mini\nhw=rev-b3-2l\nboard=BRD4265B\nchip=EFR32FG23\n"
              "fw_esp=mock\nfw_fg23=n/a\nfreq_min_hz=430000000\nfreq_max_hz=440025000\n"
              f"fs_in_hz={FS_IN}\ndecim_list=2,4,8,16,24,32,48,64\ngain_shift_max={a.gain_shift_max}\n"
              f"rssi=1\nspectrum=1\ntx=0\nmode={'USB' if usb_mode else 'STANDALONE'}\n")


def reply(text):
    queue_msg(MSG_RESPONSE, text)


def schedule_restart(new_decim, delay):
    global restart_at, restart_decim
    restart_at = time.monotonic() + delay
    restart_decim = new_decim


def handle_line(line):
    global host_spoke, usb_mode, stat_period, freq_hz, streaming, scan
    line = line.strip()
    if not line:
        return
    host_spoke = True
    st["commands"] += 1
    log(f"<- '{line}'")
    if line == "CAP?":
        hello(); return
    if line == "VER?":
        reply("OK\nfw_esp=mock\nfw_fg23=n/a\nboard=BRD4265B\n"); return
    if line.startswith("MODE"):
        p = line[4:].strip()
        if p == "USB":
            usb_mode = True; reply("OK\nmode=USB\n")
        elif p == "STANDALONE":
            usb_mode = False; reply("OK\nmode=STANDALONE\n")
        else:
            reply("ERR usage: MODE USB | MODE STANDALONE\n")
        return
    if line.startswith("STAT"):
        try:
            s = int(line[4:].strip() or "0")
        except ValueError:
            s = -1
        if s < 0 or s > 3600:
            reply("ERR usage: STAT <seconds>, 0 = off\n"); return
        stat_period = float(s); reply(f"OK\nstat_period_s={s}\n"); return
    if line.startswith("FH"):
        try:
            hz = int(line[2:].strip())
        except ValueError:
            hz = 0
        if hz < 100000000 or hz > 1000000000:
            reply("ERR FH: 100000000..1000000000 Hz\n"); return
        khz, rem = hz // 1000, hz - (hz // 1000) * 1000
        if rem > 500:
            khz += 1; rem -= 1000
        ticks = int(round(rem / TICK_HZ))
        got = khz * 1000 + int(round(ticks * TICK_HZ))
        freq_hz = got
        reply(f"OK\nf={got} (F{khz} + o{ticks})\n")
        schedule_restart(decim, 0.05)  # F stops and restarts the FG23 stream (seq -> 0)
        return
    if line[0] == "R" and len(line) > 1 and (line[1] == " " or line[1].isdigit()):
        try:
            sps = int(line[1:].strip())
        except ValueError:
            sps = 0
        if sps <= 0:
            reply("ERR usage: R <sps>\n"); return
        R = int(math.floor(FS_IN / sps + 0.5))
        if R < 3: R = 2
        elif R < 6: R = 4
        else: R = max(8, ((R + 4) // 8) * 8)
        R = min(R, 4096)
        reply(f"OK\nfs={FS_IN // R} decim={R}\n")
        if R != decim:
            schedule_restart(R, 0.15)
        return
    if line[0] == "G" and len(line) > 1 and (line[1] == " " or line[1].isdigit()):
        reply("ERR G: digital gain shift is compile-time on the FG23 (IQ_AUTOSTART_SHIFT); not yet runtime\n")
        return
    c = line[0]
    if c in REJECT:
        reply(f"ERR '{c}' is not part of the host API (TX/diagnostics)\n"); return
    if c in FORWARD:
        if c == "F":
            freq_hz = int(line[1:] or 0) * 1000
            reply(f"OK\nforwarded (rate-limited): {line}\n")
            schedule_restart(decim, 0.05)
            return
        if c == "W":
            scan = None if line == "W0" else line
            reply(f"OK\nforwarded: {line}\n"); return
        if c == "i":
            streaming = not streaming
            reply(f"OK\nforwarded: {line}\n"); return
        reply(f"OK\nforwarded: {line}\n"); return
    reply(f"ERR unknown command: {line}\n")


def lost_rule(s):
    if a.drop_every > 0:
        m, base = s % a.drop_every, a.drop_every // 2
        if base <= m < base + a.drop_burst:
            return "drop"
    if a.cut_every > 0 and s % a.cut_every == 7:
        return "cut"
    return None


k256 = np.arange(256, dtype=np.float64)


def iq_block(s):
    fs = FS_IN / decim
    n = s * 256 + k256
    ph = 2.0 * np.pi * a.tone * n / fs
    v = np.empty(512, dtype="<i2")
    v[0::2] = np.round(a.amp * np.cos(ph)).astype(np.int16)
    v[1::2] = np.round(a.amp * np.sin(ph)).astype(np.int16)
    fsfield = FS_IN | ((a.rssi & 0xFFF) << 20)
    return header(MAGIC_IQB2, s, 256, decim, fsfield) + v.tobytes()


def spc_block():
    nbin = 512
    payload = struct.pack("<IhH", 1000000, -130, 80) + bytes((i * 7) & 0xFF for i in range(nbin))
    payload += bytes(BLK - HDR - len(payload))
    return header(MAGIC_SPC1, st["spc"], nbin, 0x0001, freq_hz) + payload


def put_block(b):
    global out
    if not connected or len(out) + len(b) > OUT_LIMIT:
        st["usb_drop"] += 1
        return
    out += b


def finish(*_):
    if a.stats:
        with open(a.stats, "w") as f:
            json.dump(st, f)
    log("stats", st)
    try:
        os.unlink(a.link)
    except OSError:
        pass
    sys.exit(0)


signal.signal(signal.SIGTERM, finish)
signal.signal(signal.SIGINT, finish)

while True:
    now = time.monotonic()

    # host -> device lines
    try:
        d = os.read(master, 4096)
    except BlockingIOError:
        d = b""
    except OSError as e:
        d = b""
        if e.errno != errno.EIO:
            raise
    if d and not a.v1:
        for ch in d:
            if ch in (10, 13):
                if rx_line:
                    handle_line(rx_line.decode(errors="replace"))
                rx_line = bytearray()
            elif len(rx_line) < 95:
                rx_line.append(ch)
            else:
                rx_line = bytearray()

    # pending FG23 stream restart (F / R): seq restarts at 0, a few ms gap
    if restart_at is not None and now >= restart_at:
        decim = restart_decim
        seq = 0
        epoch_t = now + 0.005
        epoch_blocks = 0
        restart_at = None
        streaming = True
        st["restarts"] += 1
        log(f"stream (re)start R={decim} ({FS_IN / decim:.0f} sps)")

    # produce blocks that are due
    if now >= epoch_t:
        due = int((now - epoch_t) * (FS_IN / decim) / 256) - epoch_blocks
        for _ in range(max(0, due)):
            epoch_blocks += 1
            if not streaming:
                continue
            if scan:
                if epoch_blocks % 10 == 0:  # ~scan line rate, not I/Q
                    put_block(spc_block()); st["spc"] += 1
                continue
            s = seq
            seq += 1
            st["produced"] += 1
            blk_window.append(now)
            r = lost_rule(s)
            if r == "drop":
                st["dropped"] += 1
                continue
            b = iq_block(s)
            if r == "cut":
                st["cut"] += 1
                b = b[:512] + b[640:]
            put_block(b)
            if a.spc_every and st["produced"] % a.spc_every == 0:
                put_block(spc_block()); st["spc"] += 1

    # periodic status / errors (only once the host has spoken, as usb_host.cpp)
    if not a.v1 and connected and host_spoke and stat_period > 0 and now - last_stat >= stat_period:
        last_stat = now
        blk_window = [t for t in blk_window if now - t <= 2.0]
        queue_msg(MSG_STATUS, f"blk_s={len(blk_window) / 2.0:.1f} sps={FS_IN // decim} lost=0 rssi={a.rssi} "
                              f"freq_khz={freq_hz // 1000} heap_min=81 usb_drop={st['usb_drop']} "
                              f"mode={'USB' if usb_mode else 'STANDALONE'}\n")
    if not a.v1 and connected and a.err_every > 0 and now - last_err >= a.err_every:
        last_err = now
        queue_msg(MSG_ERROR, f"usbdrop={st['usb_drop']} (mock warning)\n")

    # device -> host
    hup = any(ev & select.POLLHUP for _, ev in poller.poll(0))
    if not hup and not connected:
        connected = True
        out = bytearray()
        log("host connected")
        if not a.v1:
            hello()
    elif hup and connected:
        connected = False
        out = bytearray()
        log("host gone")
    while out and connected:
        try:
            w = os.write(master, out[:65536])
        except BlockingIOError:
            break
        except OSError as e:
            if e.errno == errno.EIO:
                break
            raise
        if w <= 0:
            break
        del out[:w]

    time.sleep(0.002)
