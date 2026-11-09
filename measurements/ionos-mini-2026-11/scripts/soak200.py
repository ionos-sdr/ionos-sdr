#!/usr/bin/env python3
# 200 ksps 16-bit IQ over TinyUSB through the hub chain: N minutes, sequence-checked.
import serial, serial.tools.list_ports as lp, struct, sys, time, re, threading
MIN = float(sys.argv[1]) if len(sys.argv) > 1 else 30
port = next((p.device for p in lp.comports() if p.vid == 0x303A), "COM7")
h = serial.Serial(); h.port = port; h.baudrate = 115200; h.timeout = 0.02; h.rts = False; h.dtr = True; h.open()
try: h.set_buffer_size(rx_size=1 << 22)
except Exception: pass
c = serial.Serial(); c.port = "COM4"; c.baudrate = 115200; c.timeout = 0.05; c.dtr = False; c.rts = False; c.open()
T0 = time.time(); ESP = {"drop": 0, "lines": 0, "lost": 0}; RUN = [True]
def con_reader():
    acc = b""
    while RUN[0]:
        d = c.read(4096)
        if not d: continue
        acc += d
        while b"\n" in acc:
            line, acc = acc.split(b"\n", 1)
            m = re.search(rb"LOST=(\d+).*USB +[0-9.]+ kB/s\(-(\d+)\)", line)
            if m:
                ESP["lines"] += 1; n = int(m.group(2)) if time.time() - T0 > 10 else 0; ESP["drop"] += n; ESP["lost"] = int(m.group(1))
                if n and time.time() - T0 > 10: print("  [%.1f s] ESP usb_drop +%d" % (time.time() - T0, n), flush=True)
threading.Thread(target=con_reader, daemon=True).start()
h.write(b"MODE USB\n"); time.sleep(0.5); h.write(b"R 200000\n"); time.sleep(3); h.read(1 << 22); h.reset_input_buffer()
buf = bytearray(); seq = None; tot = {"iq": 0, "spc": 0, "msg": 0, "resync": 0, "lost": 0, "bytes": 0, "gaps": 0}
rep = time.time() + 60; last_b = 0; last_t = time.time(); end = time.time() + MIN * 60
while time.time() < end:
    d = h.read(1 << 16)
    if d: buf += d; tot["bytes"] += len(d)
    o = 0; n = len(buf)
    while n - o >= 1040:
        m = bytes(buf[o:o + 4])
        if m == b"IQB2":
            tot["iq"] += 1; s = struct.unpack_from("<I", buf, o + 4)[0]
            if seq is not None and s != (seq + 1) & 0xFFFFFFFF and time.time() - T0 > 10:
                k = (s - seq - 1) & 0xFFFFFFFF; tot["lost"] += k; tot["gaps"] += 1
                print("  [%.1f s] SEQ GAP +%d" % (time.time() - T0, k), flush=True)
            seq = s
        elif m == b"SPC1": tot["spc"] += 1
        elif m == b"MSG1": tot["msg"] += 1
        else: o += 1; tot["resync"] += 1; continue
        o += 1040
    del buf[:o]
    if time.time() >= rep:
        now = time.time()
        print("%5.1f min  %.1f kB/s  iq %d lost %d (gaps %d) resync %d | ESP usb_drop %d LOST %d" % (
            (now - T0) / 60, (tot["bytes"] - last_b) / (now - last_t) / 1000, tot["iq"], tot["lost"],
            tot["gaps"], tot["resync"], ESP["drop"], ESP["lost"]), flush=True)
        last_b = tot["bytes"]; last_t = now; rep += 60
RUN[0] = False
print("END %.1f min: iq %d lost %d (%.5f %%) gaps %d resync %d ESP usb_drop %d" % ((time.time() - T0) / 60,
      tot["iq"], tot["lost"], 100.0 * tot["lost"] / max(1, tot["iq"] + tot["lost"]), tot["gaps"], tot["resync"], ESP["drop"]), flush=True)
h.write(b"R 50000\n"); time.sleep(1); h.close(); c.close()
