#!/usr/bin/env python3
# 200 ksps overnight soak with full logging until a wall-clock end time (default 07:00).
# soak_full_<stamp>.log : per-second host stats + every event (seq gap, resync, ESP drop)
# console_<stamp>.log   : every COM4 console line, wall-clock stamped (incl. US stats every 60 s)
import serial, serial.tools.list_ports as lp, struct, sys, time, re, threading, datetime as dt
END_H = int(sys.argv[1]) if len(sys.argv) > 1 else 7
now = dt.datetime.now(); end_dt = now.replace(hour=END_H, minute=0, second=0, microsecond=0)
if end_dt <= now: end_dt += dt.timedelta(days=1)
stamp = now.strftime("%Y%m%d_%H%M")
LOG = open("soak_full_%s.log" % stamp, "a", encoding="utf-8", buffering=1)
CLOG = open("console_%s.log" % stamp, "a", encoding="utf-8", buffering=1)
def ts(): return dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
def log(s): LOG.write("%s %s\n" % (ts(), s)); print("%s %s" % (ts(), s), flush=True)
port = next((p.device for p in lp.comports() if p.vid == 0x303A), "COM7")
h = serial.Serial(); h.port = port; h.baudrate = 115200; h.timeout = 0.02; h.rts = False; h.dtr = True; h.open()
try: h.set_buffer_size(rx_size=1 << 22)
except Exception: pass
c = serial.Serial(); c.port = "COM4"; c.baudrate = 115200; c.timeout = 0.05; c.dtr = False; c.rts = False; c.open()
T0 = time.time(); ESP = {"drop": 0, "lost": 0}; RUN = [True]; CW = threading.Lock()
def con_reader():
    acc = b""
    while RUN[0]:
        d = c.read(4096)
        if not d: continue
        acc += d
        while b"\n" in acc:
            line, acc = acc.split(b"\n", 1); txt = line.decode("latin1").rstrip()
            CLOG.write("%s %s\n" % (ts(), txt))
            m = re.search(r"LOST=(\d+).*USB +[0-9.]+ kB/s\(-(\d+)\)", txt)
            if m:
                n = int(m.group(2)) if time.time() - T0 > 10 else 0
                ESP["drop"] += n; ESP["lost"] = int(m.group(1))
                if n: log("EVENT ESP usb_drop +%d (total %d)" % (n, ESP["drop"]))
            if txt.startswith(("USBGAP", "USBST", "USBHIST")): log("STAT " + txt[:240])
            if any(k in txt for k in ("rst:", "Guru", "panic", "abort", "WDT", "Brownout", "BOOT")): log("EVENT console: " + txt[:200])
def con_cmd(s):
    with CW: c.write((s + "\n").encode())
threading.Thread(target=con_reader, daemon=True).start()
log("START port %s, until %s" % (port, end_dt.strftime("%Y-%m-%d %H:%M")))
h.write(b"MODE USB\n"); time.sleep(0.5); h.write(b"R 200000\n"); time.sleep(3); h.read(1 << 22); h.reset_input_buffer()
for extra in sys.argv[2:]:
    con_cmd(extra); log("console cmd: " + extra); time.sleep(1.0)
con_cmd("US")
buf = bytearray(); seq = None
tot = {"iq": 0, "spc": 0, "msg": 0, "resync": 0, "lost": 0, "bytes": 0, "gaps": 0}
sec = {"bytes": 0, "maxgap": 0.0, "reads": 0}; last_read = time.time(); next_s = time.time() + 1; next_us = time.time() + 60
rs_run = 0
while dt.datetime.now() < end_dt:
    d = h.read(1 << 16); t = time.time()
    if d:
        g = t - last_read; last_read = t
        if g > sec["maxgap"]: sec["maxgap"] = g
        if g > 0.25: log("EVENT host read stall %.0f ms" % (g * 1000))
        buf += d; tot["bytes"] += len(d); sec["bytes"] += len(d); sec["reads"] += 1
    o = 0; n = len(buf)
    while n - o >= 1040:
        m = bytes(buf[o:o + 4])
        if m == b"IQB2":
            if rs_run: log("EVENT resync skipped %d bytes before seq %d" % (rs_run, struct.unpack_from("<I", buf, o + 4)[0])); rs_run = 0
            tot["iq"] += 1; s = struct.unpack_from("<I", buf, o + 4)[0]
            if seq is not None and s != (seq + 1) & 0xFFFFFFFF and t - T0 > 10:
                k = (s - seq - 1) & 0xFFFFFFFF
                if k < 1 << 31: tot["lost"] += k; tot["gaps"] += 1; log("EVENT SEQ GAP %d -> %d (+%d)" % (seq, s, k))
                else: log("EVENT SEQ BACK %d -> %d" % (seq, s))
            seq = s
        elif m == b"SPC1": tot["spc"] += 1
        elif m == b"MSG1": tot["msg"] += 1
        else: o += 1; tot["resync"] += 1; rs_run += 1; continue
        o += 1040
    del buf[:o]
    if t >= next_s:
        LOG.write("%s SEC %.1f kB/s reads %d maxgap %.0f ms | iq %d lost %d gaps %d resync %d | ESP drop %d LOST %d\n" % (
            ts(), sec["bytes"] / 1000, sec["reads"], sec["maxgap"] * 1000, tot["iq"], tot["lost"], tot["gaps"],
            tot["resync"], ESP["drop"], ESP["lost"]))
        sec = {"bytes": 0, "maxgap": 0.0, "reads": 0}; next_s += 1
        if next_s < t: next_s = t + 1
    if t >= next_us:
        con_cmd("US"); next_us += 60
        el = (t - T0) / 60
        print("%s %.1f min iq %d lost %d gaps %d resync %d ESP drop %d" % (ts(), el, tot["iq"], tot["lost"], tot["gaps"], tot["resync"], ESP["drop"]), flush=True)
RUN[0] = False
log("END %.1f min: iq %d lost %d (%.5f %%) gaps %d resync %d ESP usb_drop %d" % ((time.time() - T0) / 60,
    tot["iq"], tot["lost"], 100.0 * tot["lost"] / max(1, tot["iq"] + tot["lost"]), tot["gaps"], tot["resync"], ESP["drop"]))
h.write(b"R 50000\n"); time.sleep(1); h.close(); c.close()
