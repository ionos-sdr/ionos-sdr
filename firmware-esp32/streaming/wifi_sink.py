#!/usr/bin/env python3
"""
wifi_sink.py - a wifi_bench.cpp fuggetlen meropontja

HA7DCD / Kolibri.  Amit ez mer, es amit az ESP nem tud merni:

  * a TENYLEGES beerkezesi rata a PC/telefon oldalan,
  * a keretek kozti szunetek eloszlasa - TCP nem VESZIT, hanem MEGALL,
    es az SDR++ szaggatasa pontosan ez: a >50 ms-os lyukak szama,
  * az adat epsege (minta-ellenorzes), ha valaha gyanu merul fel.

A lepcso-indexet a keretfejlec hozza, tehat a nyelo FUGGETLENUL is tud
lepcsonkent osszesiteni - nem kell a ket oldalt kezzel osszefesulni.
Opcionalisan a soros konzolt is olvassa (--serial COM10), es a BENCH-STEP
sorokbol beemeli az SPI-vesztesegszamlalokat ugyanabba a tablazatba.

Hasznalat:
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

# ugyanaz a minta, mint a wb_fill_pattern()-ben
PATTERN = bytes(((i * 31 + 7) & 0xFF) for i in range(MAX_BLK))

STOP = threading.Event()


# --------------------------------------------------------------------------
class StepAcc:
    """Egy lepcso osszesitoje a nyelo oldalarol."""

    def __init__(self, step, offered_kBps):
        self.step = step
        self.offered = offered_kBps
        self.bytes = 0
        self.frames = 0
        self.bad = 0
        self.seq_gaps = 0
        self.t_first = None
        self.t_last = None
        self.gaps = []          # keretek kozti szunetek [ms]

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
    """A soros konzolrol beemeli a BENCH-STEP sorokat (opcionalis)."""
    try:
        import serial  # pyserial
    except ImportError:
        print("! pyserial nincs telepitve, a --serial kimarad "
              "(pip install pyserial)", file=sys.stderr)
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as exc:
        print(f"! soros port nem nyithato ({port}): {exc}", file=sys.stderr)
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
            if "lep" in d:
                esp_steps[int(d["lep"])] = d
    ser.close()


# --------------------------------------------------------------------------
def run(args):
    esp_steps = {}
    if args.serial:
        threading.Thread(
            target=serial_reader,
            args=(args.serial, args.baud, esp_steps, not args.quiet),
            daemon=True).start()

    print(f"-> csatlakozas {args.host}:{args.port} ...")
    s = socket.create_connection((args.host, args.port), timeout=10)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    if args.rcvbuf:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.rcvbuf)
    s.settimeout(args.timeout)
    print("   kapcsolat all, varom a kereteket "
          f"(ablak {args.window}s, lyuk-kuszob {args.gap_ms} ms)")

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
                print("! idotullepes - az ESP nem kuld "
                      "(vege a menetnek, vagy megallt)")
                break
            if not chunk:
                print("-> a kapcsolat lezarult")
                break
            buf += chunk
            total_bytes += len(chunk)
            now = time.perf_counter()

            # --- keretek kifejtese ---
            while True:
                if len(buf) < HDR_LEN:
                    break
                if bytes(buf[:4]) != MAGIC:
                    # ujraszinkron: keressuk a kovetkezo magicet
                    idx = buf.find(MAGIC, 1)
                    if idx < 0:
                        del buf[:max(0, len(buf) - 3)]
                        break
                    print(f"! szinkronvesztes, {idx} bajt eldobva")
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
                    print(f"\n=== lepcso {step}: felkinalt "
                          f"{rate} kB/s ({rate/4:.0f} ksps int16) ===")

                if flags & 1:          # bemelegites: nem szamit sehova
                    last_seq = seq
                    win_t0 = now       # az ablak is csak utana indul
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

            # --- ablak-riport ---
            dt = now - win_t0
            if dt >= args.window:
                kBps = win_bytes / 1000.0 / dt
                if not args.quiet:
                    print(f"   {kBps:8.1f} kB/s  "
                          f"= {kBps*1000/4:8.0f} sps int16 "
                          f"/ {kBps*1000/2:8.0f} sps int8  "
                          f"ker={win_frames:6d}  lyukmax={win_gapmax:7.2f} ms  "
                          f"lyuk>{args.gap_ms}ms={win_drop}")
                win_t0, win_bytes, win_frames = now, 0, 0
                win_gapmax, win_drop = 0.0, 0
    except KeyboardInterrupt:
        print("\n-> megszakitva")
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
    print(f"--- lepcso {r['step']} osszegzes: "
          f"felkinalt {r['offered_kBps']} kB/s -> elert {r['ach_kBps']} kB/s "
          f"({r['sps16']} sps int16), lyuk p99={r['gap_p99_ms']} ms "
          f"max={r['gap_max_ms']} ms, kiesesek={r['dropouts']}"
          + (f", SPI lost={r['esp_lost']}" if r["esp_lost"] != "" else ""))


def _final(rows, args, dur, total):
    print("\n" + "=" * 78)
    print(f"osszesen {total/1e6:.2f} MB / {dur:.1f} s = "
          f"{total/1000.0/dur:.1f} kB/s atlag")
    if not rows:
        print("nem jott ertekelheto lepcso")
        return

    hdr = ("lep", "felk.kB/s", "elert", "sps16", "lyuk_p99", "lyuk_max",
           "kieses", "SPIlost", "dup", "ovf", "eagain")
    print("\n{:>4} {:>10} {:>9} {:>9} {:>9} {:>9} {:>7} {:>8} {:>5} {:>5} {:>7}"
          .format(*hdr))
    for r in rows:
        print("{:>4} {:>10} {:>9} {:>9} {:>9} {:>9} {:>7} {:>8} {:>5} {:>5} {:>7}"
              .format(r["step"], r["offered_kBps"], r["ach_kBps"], r["sps16"],
                      r["gap_p99_ms"], r["gap_max_ms"], r["dropouts"],
                      r["esp_lost"], r["esp_dup"], r["esp_ovf"], r["esp_eagain"]))

    # --- a terd: a legnagyobb rata, ahol meg minden tiszta ---
    clean = [r for r in rows
             if r["dropouts"] == 0 and r["bad"] == 0
             and (r["esp_lost"] in ("", 0))]
    if clean:
        best = max(clean, key=lambda r: r["ach_kBps"])
        print(f"\n>> TERD: {best['ach_kBps']} kB/s tisztan "
              f"= {best['sps16']} sps int16 / {best['sps8']} sps int8")
    else:
        print("\n>> minden lepcson volt kieses vagy SPI-veszteseg - "
              "kezdd lejjebb, vagy nezd meg a Bset core/prio ertekeket")

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
        print("! matplotlib nincs, a --plot kimarad")
        return
    off = [r["offered_kBps"] for r in rows]
    ach = [r["ach_kBps"] for r in rows]
    drop = [r["dropouts"] for r in rows]
    lost = [r["esp_lost"] if r["esp_lost"] != "" else 0 for r in rows]

    fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(off, ach, "o-", label="elert")
    ax[0].plot(off, off, "k--", lw=0.8, label="ideal")
    ax[0].set_ylabel("kB/s")
    ax[0].grid(alpha=0.3)
    ax[0].legend()
    ax[0].set_title("WiFi atviteli plafon es a lanc ara")

    ax[1].plot(off, drop, "s-", label="kliens-kiesesek")
    ax[1].plot(off, lost, "^-", label="SPI blokkveszteseg")
    ax[1].set_xlabel("felkinalt rata [kB/s]")
    ax[1].set_ylabel("darab / lepcso")
    ax[1].grid(alpha=0.3)
    ax[1].legend()
    plt.tight_layout()
    plt.show()


# --------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="wifi_bench nyelo")
    p.add_argument("host")
    p.add_argument("--port", type=int, default=7777)
    p.add_argument("--window", type=float, default=1.0,
                   help="riportablak masodpercben")
    p.add_argument("--gap-ms", type=float, default=50.0,
                   help="ennel hosszabb keretszunet = kieses (SDR++ szaggatas)")
    p.add_argument("--check", action="store_true",
                   help="payload-minta ellenorzese (lassabb)")
    p.add_argument("--recv", type=int, default=65536,
                   help="recv() puffermeret")
    p.add_argument("--rcvbuf", type=int, default=0,
                   help="SO_RCVBUF (0 = rendszer alapertelmezes)")
    p.add_argument("--timeout", type=float, default=15.0)
    p.add_argument("--csv", default=None)
    p.add_argument("--serial", default=None,
                   help="ESP konzol portja (pl. COM10) a BENCH-STEP sorokhoz")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--plot", action="store_true")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    signal.signal(signal.SIGINT, lambda *_: STOP.set())
    run(args)


if __name__ == "__main__":
    main()
