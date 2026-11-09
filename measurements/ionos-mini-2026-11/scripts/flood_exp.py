#!/usr/bin/env python3
# Flood throughput vs TinyUSB task priority and vs the FG23 stream running.
import serial, serial.tools.list_ports as lp, time
port = next((p.device for p in lp.comports() if p.vid == 0x303A), "COM7")
c = serial.Serial(); c.port = "COM4"; c.baudrate = 115200; c.timeout = 0.1; c.dtr = False; c.rts = False; c.open()
h = serial.Serial(); h.port = port; h.baudrate = 115200; h.timeout = 0.1; h.rts = False; h.dtr = True; h.open()
try: h.set_buffer_size(rx_size=1 << 22)
except Exception: pass
def con(cmd, wait=0.5):
    c.reset_input_buffer(); c.write((cmd + "\n").encode()); t0 = time.time(); o = b""
    while time.time() - t0 < wait: o += c.read(4096); h.read(h.in_waiting or 1)
    return [l for l in o.decode("latin1").splitlines() if l.startswith("USB") or "blk/s" in l]
def flood(label, sec=15):
    con("US"); con("UF%d" % sec, 0.1)
    t0 = time.time(); tot = 0
    while time.time() - t0 < sec: tot += len(h.read(1 << 20))
    k = tot / (time.time() - t0) / 1000
    st = con("US", 0.8)
    print("%-28s %.1f kB/s" % (label, k), flush=True)
    for l in st: print("    ", l[:200], flush=True)
    time.sleep(1); h.read(1 << 20)
h.write(b"MODE USB\n"); time.sleep(0.5); h.write(b"R 200000\n"); time.sleep(2)
for p in ("5", "24"):
    con("UP" + p); flood("prio %s, stream 200k on" % p)
h.write(b"i\n"); time.sleep(1.0); h.read(1 << 20)        # FG23 stream off
print(con("s", 1.0)[:1])
for p in ("5", "24"):
    con("UP" + p); flood("prio %s, stream OFF" % p)
h.write(b"R 50000\n"); time.sleep(2); con("UP5")
h.close(); c.close()
