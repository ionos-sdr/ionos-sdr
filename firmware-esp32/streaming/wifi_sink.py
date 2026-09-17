#!/usr/bin/env python3
"""
wifi_sink.py - the independent measurement point for wifi_bench.cpp

HA7DCD.  What this measures, and what the ESP cannot:

  * the ACTUAL arrival rate on the PC/phone side,
  * the distribution of inter-frame gaps - TCP does not LOSE, it STALLS,
    and SDR++ stuttering is exactly that: the number of >50 ms gaps,
  * data integrity (pattern check), should suspicion ever arise.

The step index comes in the frame header, so the sink can aggregate per
step INDEPENDENTLY - no manual merging of the two sides is needed.
Optionally it also reads the serial console (--serial COM10) and pulls the
SPI loss counters from the BENCH-STEP lines into the same table.

Usage:
    python wifi_sink.py 192.168.1.50
    python wifi_sink.py 192.168.1.50 --port 7777 --csv bench.csv --serial COM10
    python wifi_sink.py 192.168.1.50 --check --gap-ms 20 --plot
"""

import argparse
import csv
import re
import signal
import socket
import statistics
import struct
import sys
import threading
import time

HDR_FMT = "<4sIII HH I"          # magic, seq, len, t_us, step, flags, rate_kBps
HDR_LEN = struct.calcsize(HDR_FMT)
assert HDR_LEN == 24, HDR_LEN
MAGIC = b"WBN1"
MAX_BLK = 8192

# the same pattern as in wb_fill_pattern()
PATTERN = bytes(((i * 31 + 7) & 0xFF) for i in range(MAX_BLK))

STOP = threading.Event()


# --------------------------------------------------------------------------
class StepAcc:
    """Aggregator of one step from the sink side."""

    def __init__(self, step, offered_kBps):
        self.step = step
        self.offered = offered_kBps
        self.bytes = 0
        self.frames = 0
        self.bad = 0
        self.seq_gaps = 0
        self.t_first = None
        self.t_last = None
        self.gaps = []          # inter-frame gaps [ms]

    def add(self, nbytes, t, gap_ms, bad):
        if self.t_first is None:
            self.t_first = t
        self.t_last = t
        self.bytes += nbytes
        self.frames += 1
        if bad:
            self.bad += 1
        if gap_ms is not None:
            self.gaps.append(gap_ms)

    def summary(self, gap_thr_ms):
        dur = (self.t_last - self.t_first) if self.t_first is not None else 0.0
        kBps = (self.bytes / 1000.0 / dur) if dur > 0 else 0.0
        g = self.gaps or [0.0]
        gs = sorted(g)
        p99 = gs[min(len(gs) - 1, int(0.99 * len(gs)))]
        return {
            "step": self.step,
            "offered_kBps": self.offered,
            "ach_kBps": round(kBps, 1),
            "sps16": round(kBps * 1000.0 / 4.0),
            "sps8": round(kBps * 1000.0 / 2.0),
            "sec": round(dur, 1),
            "frames": self.frames,
            "bad": self.bad,
            "seq_gaps": self.seq_gaps,
            "gap_med_ms": round(statistics.median(g), 2),
            "gap_p99_ms": round(p99, 2),
            "gap_max_ms": round(max(g), 2),
            "dropouts": sum(1 for x in g if x > gap_thr_ms),
        }


# --------------------------------------------------------------------------
def serial_reader(port, baud, esp_steps, echo):
    """Pulls the BENCH-STEP lines from the serial console (optional)."""
    try:
        import serial  # pyserial
    except ImportError:
        print("! pyserial is not installed, --serial is skipped "
              "(pip install pyserial)", file=sys.stderr)
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as exc:
        print(f"! cannot open serial port ({port}): {exc}", file=sys.stderr)
        return
    kv = re.compile(r"(\w+)=(-?[\d.]+)")
    while not STOP.is_set():
        try:
            line = ser.readline().decode("utf-8", "replace").strip()
        except Exception:
            break
        if not line:
            continue
        if echo and ("BENCH" in line):
            print("   [esp] " + line)
        if line.startswith("BENCH-STEP:"):
            d = {k: (float(v) if "." in v else int(v))
                 for k, v in kv.findall(line)}
            if "step" in d:
                esp_steps[int(d["step"])] = d
    ser.close()


# --------------------------------------------------------------------------
def run(args):
    esp_steps = {}
    if args.serial:
        threading.Thread(
            target=serial_reader,
            args=(args.serial, args.baud, esp_steps, not args.quiet),
            daemon=True).start()

    print(f"-> connecting to {args.host}:{args.port} ...")
    s = socket.create_connection((args.host, args.port), timeout=10)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    if args.rcvbuf:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.rcvbuf)
    s.settimeout(args.timeout)
    print("   connected, waiting for frames "
          f"(window {args.window}s, gap threshold {args.gap_ms} ms)")

    buf = bytearray()
    steps = {}                  # step -> StepAcc
    order = []
    cur_step = None
    last_seq = None
    t_prev = None

    win_t0 = time.perf_counter()
    win_bytes = 0
    win_frames = 0
    win_gapmax = 0.0
    win_drop = 0

    rows = []
    total_bytes = 0
    t_start = time.perf_counter()

    try:
        while not STOP.is_set():
            try:
                chunk = s.recv(args.recv)
            except socket.timeout:
                print("! timeout - the ESP is not sending "
                      "(run finished, or it stalled)")
                break
            if not chunk:
                print("-> connection closed")
                break
            buf += chunk
            total_bytes += len(chunk)
            now = time.perf_counter()

            # --- frame extraction ---
            while True:
                if len(buf) < HDR_LEN:
                    break
                if bytes(buf[:4]) != MAGIC:
                    # resync: search for the next magic
                    idx = buf.find(MAGIC, 1)
                    if idx < 0:
                        del buf[:max(0, len(buf) - 3)]
                        break
                    print(f"! sync lost, {idx} bytes discarded")
                    del buf[:idx]
                    continue
                magic, seq, ln, t_us, step, flags, rate = struct.unpack(
                    HDR_FMT, bytes(buf[:HDR_LEN]))
                if ln < HDR_LEN or ln > MAX_BLK:
                    del buf[:4]
                    continue
                if len(buf) < ln:
                    break

                frame = bytes(buf[:ln])
                del buf[:ln]

                bad = False
                if args.check:
                    if frame[HDR_LEN:] != PATTERN[:ln - HDR_LEN]:
                        bad = True

                gap_ms = None if t_prev is None else (now - t_prev) * 1000.0
                t_prev = now

                if step not in steps:
                    steps[step] = StepAcc(step, rate)
                    order.append(step)
                    if cur_step is not None:
                        _print_step(steps[cur_step], args, esp_steps, rows)
                    cur_step = step
                    last_seq = None
                    print(f"\n=== step {step}: offered "
                          f"{rate} kB/s ({rate/4:.0f} ksps int16) ===")

                if flags & 1:          # warm-up: counted nowhere
                    last_seq = seq
                    win_t0 = now       # the window also starts only afterwards
                    win_bytes = win_frames = win_drop = 0
                    win_gapmax = 0.0
                    t_prev = None
                    continue

                if last_seq is not None and seq != last_seq + 1:
                    steps[step].seq_gaps += 1
                last_seq = seq

                steps[step].add(ln, now, gap_ms, bad)

                win_bytes += ln
                win_frames += 1
                if gap_ms is not None:
                    win_gapmax = max(win_gapmax, gap_ms)
                    if gap_ms > args.gap_ms:
                        win_drop += 1

            # --- window report ---
            dt = now - win_t0
            if dt >= args.window:
                kBps = win_bytes / 1000.0 / dt
                if not args.quiet:
                    print(f"   {kBps:8.1f} kB/s  "
                          f"= {kBps*1000/4:8.0f} sps int16 "
                          f"/ {kBps*1000/2:8.0f} sps int8  "
                          f"frames={win_frames:6d}  gapmax={win_gapmax:7.2f} ms  "
                          f"gap>{args.gap_ms}ms={win_drop}")
                win_t0, win_bytes, win_frames = now, 0, 0
                win_gapmax, win_drop = 0.0, 0
    except KeyboardInterrupt:
        print("\n-> interrupted")
    finally:
        STOP.set()
        s.close()

    if cur_step is not None:
        _print_step(steps[cur_step], args, esp_steps, rows)

    _final(rows, args, time.perf_counter() - t_start, total_bytes)
    return rows


def _print_step(acc, args, esp_steps, rows):
    r = acc.summary(args.gap_ms)
    e = esp_steps.get(acc.step, {})
    for k in ("lost", "dup", "ovf", "spydrop", "eagain", "tmax", "dtmax", "ring"):
        r["esp_" + k] = e.get(k, "")
    rows.append(r)
    print(f"--- step {r['step']} summary: "
          f"offered {r['offered_kBps']} kB/s -> achieved {r['ach_kBps']} kB/s "
          f"({r['sps16']} sps int16), gap p99={r['gap_p99_ms']} ms "
          f"max={r['gap_max_ms']} ms, dropouts={r['dropouts']}"
          + (f", SPI lost={r['esp_lost']}" if r["esp_lost"] != "" else ""))


def _final(rows, args, dur, total):
    print("\n" + "=" * 78)
    print(f"total {total/1e6:.2f} MB / {dur:.1f} s = "
          f"{total/1000.0/dur:.1f} kB/s average")
    if not rows:
        print("no evaluable step received")
        return

    hdr = ("step", "off.kB/s", "achieved", "sps16", "gap_p99", "gap_max",
           "dropout", "SPIlost", "dup", "ovf", "eagain")
    print("\n{:>4} {:>10} {:>9} {:>9} {:>9} {:>9} {:>7} {:>8} {:>5} {:>5} {:>7}"
          .format(*hdr))
    for r in rows:
        print("{:>4} {:>10} {:>9} {:>9} {:>9} {:>9} {:>7} {:>8} {:>5} {:>5} {:>7}"
              .format(r["step"], r["offered_kBps"], r["ach_kBps"], r["sps16"],
                      r["gap_p99_ms"], r["gap_max_ms"], r["dropouts"],
                      r["esp_lost"], r["esp_dup"], r["esp_ovf"], r["esp_eagain"]))

    # --- the knee: the highest rate at which everything is still clean ---
    clean = [r for r in rows
             if r["dropouts"] == 0 and r["bad"] == 0
             and (r["esp_lost"] in ("", 0))]
    if clean:
        best = max(clean, key=lambda r: r["ach_kBps"])
        print(f"\n>> KNEE: {best['ach_kBps']} kB/s clean "
              f"= {best['sps16']} sps int16 / {best['sps8']} sps int8")
    else:
        print("\n>> every step had dropouts or SPI loss - "
              "start lower, or check the Bset core/prio values")

    if args.csv:
        keys = list(rows[0].keys())
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        print(f"\nCSV: {args.csv}")

    if args.plot:
        _plot(rows)


def _plot(rows):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("! matplotlib missing, --plot is skipped")
        return
    off = [r["offered_kBps"] for r in rows]
    ach = [r["ach_kBps"] for r in rows]
    drop = [r["dropouts"] for r in rows]
    lost = [r["esp_lost"] if r["esp_lost"] != "" else 0 for r in rows]

    fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(off, ach, "o-", label="achieved")
    ax[0].plot(off, off, "k--", lw=0.8, label="ideal")
    ax[0].set_ylabel("kB/s")
    ax[0].grid(alpha=0.3)
    ax[0].legend()
    ax[0].set_title("WiFi throughput ceiling and the cost to the chain")

    ax[1].plot(off, drop, "s-", label="client dropouts")
    ax[1].plot(off, lost, "^-", label="SPI block loss")
    ax[1].set_xlabel("offered rate [kB/s]")
    ax[1].set_ylabel("count / step")
    ax[1].grid(alpha=0.3)
    ax[1].legend()
    plt.tight_layout()
    plt.show()


# --------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="wifi_bench sink")
    p.add_argument("host")
    p.add_argument("--port", type=int, default=7777)
    p.add_argument("--window", type=float, default=1.0,
                   help="report window in seconds")
    p.add_argument("--gap-ms", type=float, default=50.0,
                   help="inter-frame gap longer than this = dropout (SDR++ stutter)")
    p.add_argument("--check", action="store_true",
                   help="verify the payload pattern (slower)")
    p.add_argument("--recv", type=int, default=65536,
                   help="recv() buffer size")
    p.add_argument("--rcvbuf", type=int, default=0,
                   help="SO_RCVBUF (0 = system default)")
    p.add_argument("--timeout", type=float, default=15.0)
    p.add_argument("--csv", default=None)
    p.add_argument("--serial", default=None,
                   help="ESP console port (e.g. COM10) for the BENCH-STEP lines")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--plot", action="store_true")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    signal.signal(signal.SIGINT, lambda *_: STOP.set())
    run(args)


if __name__ == "__main__":
    main()
