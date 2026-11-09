#!/usr/bin/env python3
# Gap anatomy (TX FIFO state during the ~95 ms gaps) and the EP2 FIFO size (UT<words>),
# flood and the real 200 ksps stream, with the device blk/s line (USB drops) from COM4.
import serial, serial.tools.list_ports as lp, time
port = next((p.device for p in lp.comports() if p.vid == 0x303A), "COM7")
c = serial.Serial(); c.port = "COM4"; c.baudrate = 115200; c.timeout = 0.1; c.dtr = False; c.rts = False; c.open()
h = serial.Serial(); h.port = port; h.baudrate = 115200; h.timeout = 0.02; h.rts = False; h.dtr = True; h.open()
try: h.set_buffer_size(rx_size=1 << 22)
except Exception: pass
def con(cmd, wait=0.6, keep=("USB", "EP2", "TinyUSB")):
    c.reset_input_buffer(); c.write((cmd + "\n").encode()); t0 = time.time(); o = b""
    while time.time() - t0 < wait: o += c.read(4096); h.read(1 << 16)
    return [l for l in o.decode("latin1").splitlines() if l.startswith(keep)]
def rd(sec):
    t0 = time.time(); tot = 0
    while time.time() - t0 < sec: tot += len(h.read(1 << 16))
    return tot / (time.time() - t0) / 1000
def show(ls):
    for l in ls:
        if not l.startswith("USBDWC2"): print("    ", l[:230], flush=True)
h.write(b"MODE USB\n"); time.sleep(0.5); h.write(b"R 200000\n"); rd(3)
for ut in (None, 64, 128):
    if ut: print(con("UT%d" % ut, 0.8), flush=True)
    con("US"); c.write(b"UF15\n"); k = rd(15)
    print("FLOOD fifo %s: %.1f kB/s" % (ut or "32", k), flush=True); show(con("US", 0.8))
    rd(2); con("US")
    c.reset_input_buffer(); k = rd(60)
    lines = [l for l in c.read(1 << 16).decode("latin1").splitlines() if "blk/s" in l]
    print("STREAM fifo %s: host %.1f kB/s (needs 812.5)" % (ut or "32", k), flush=True)
    show(con("US", 0.8))
    for l in lines[-3:]: print("    ", l[:200])
h.write(b"R 50000\n"); time.sleep(1)
h.close(); c.close()
