#!/usr/bin/env python3
# fg23_scan_view.py — wideband scan waterfall from the ESP32 SpyServer FFT stream,
# WITHOUT SDR++. HA7DCD 2026-08-15.
#
# Requests the same from the server as the SDR++ fg23_scan_source module in SCAN mode:
#   STREAMING_MODE = FFT_ONLY (4), FFT_FREQUENCY, FFT_DECIMATION (=SPAN Hz),
#   FFT_DISPLAY_PIXELS (=nbin), FFT_DB_OFFSET (=-floor), FFT_DB_RANGE,
#   STREAMING_ENABLED = 1
# and draws the MSG_TYPE_UINT8_FFT (301) messages with matplotlib.
#
# Usage (one line, PowerShell):
#   py -3.14 fg23_scan_view.py --host 192.168.1.50 --center 149.8e6 --span 10e6 --nbin 320
# Keys in the window: q = quit,  left/right arrow = center +-span/4,
#   up/down arrow = span x2 / :2,  r = autoscale.
#
# Dependencies: numpy, matplotlib (py -3.14 -m pip install matplotlib if missing).

import argparse, socket, struct, sys, threading, time, collections
import numpy as np

PROTO_VER = (2 << 24) | (0 << 16) | 1700
CMD_HELLO, CMD_SET_SETTING = 0, 2
S_STREAMING_MODE, S_STREAMING_ENABLED = 0, 1
S_FFT_FORMAT, S_FFT_FREQUENCY, S_FFT_DECIMATION = 200, 201, 202
S_FFT_DB_OFFSET, S_FFT_DB_RANGE, S_FFT_DISPLAY_PIXELS = 203, 204, 205
STREAM_MODE_FFT_ONLY = 4
FMT_UINT8 = 1
MSG_DEVICE_INFO, MSG_CLIENT_SYNC, MSG_UINT8_FFT = 0, 1, 301
HDR = struct.Struct("<IIIII")            # ProtocolID, MessageType, StreamType, Seq, BodySize


class SpyScan:
    def __init__(self, host, port, center, span, nbin, floor, rng):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.center, self.span, self.nbin, self.floor, self.rng = center, span, nbin, floor, rng
        self.lines = collections.deque(maxlen=400)   # most recent rows (dB)
        self.n_lines = 0
        self.dev = None
        self.alive = True
        self._cmd(CMD_HELLO, struct.pack("<I", PROTO_VER) + b"fg23_scan_view")
        self.apply()
        threading.Thread(target=self._rx, daemon=True).start()

    def _cmd(self, ctype, body):
        self.sock.sendall(struct.pack("<II", ctype, len(body)) + body)

    def _set(self, sid, val):
        self._cmd(CMD_SET_SETTING, struct.pack("<II", sid, int(val) & 0xFFFFFFFF))

    def apply(self):
        self._set(S_FFT_FORMAT, FMT_UINT8)
        self._set(S_FFT_FREQUENCY, self.center)
        self._set(S_FFT_DECIMATION, self.span)             # = SPAN Hz
        self._set(S_FFT_DISPLAY_PIXELS, self.nbin)
        self._set(S_FFT_DB_OFFSET, -self.floor)
        self._set(S_FFT_DB_RANGE, self.rng)
        self._set(S_STREAMING_MODE, STREAM_MODE_FFT_ONLY)
        self._set(S_STREAMING_ENABLED, 1)

    def stop(self):
        self.alive = False
        try:
            self._set(S_STREAMING_ENABLED, 0)
            self._set(S_STREAMING_MODE, 1)                 # IQ_ONLY -> the FG23 receives W0
            time.sleep(0.2)
            self.sock.close()
        except OSError:
            pass

    def _readn(self, n):
        buf = b""
        while len(buf) < n:
            c = self.sock.recv(n - len(buf))
            if not c:
                raise ConnectionError("disconnected")
            buf += c
        return buf

    def _rx(self):
        try:
            while self.alive:
                h = self._readn(HDR.size)
                _, mtype, _, seq, blen = HDR.unpack(h)
                body = self._readn(blen) if blen else b""
                mt = mtype & 0xFFFF
                if mt == MSG_DEVICE_INFO and blen >= 48:
                    self.dev = struct.unpack("<12I", body[:48])
                elif mt == MSG_UINT8_FFT:
                    v = np.frombuffer(body, dtype=np.uint8).astype(np.float32)
                    self.lines.append(self.floor + v * (self.rng / 255.0))
                    self.n_lines += 1
        except (OSError, ConnectionError) as e:
            if self.alive:
                print("rx: connection lost:", e, file=sys.stderr)
            self.alive = False


def main():
    ap = argparse.ArgumentParser(description="FG23 scan waterfall from the SpyServer FFT stream")
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--center", type=float, default=149.8e6, help="Hz")
    ap.add_argument("--span", type=float, default=10e6, help="Hz")
    ap.add_argument("--nbin", type=int, default=320)
    ap.add_argument("--floor", type=int, default=-130, help="dBm (bin level 0)")
    ap.add_argument("--range", type=int, default=100, dest="rng", help="dB")
    ap.add_argument("--rows", type=int, default=200, help="waterfall rows")
    ap.add_argument("--nogui", action="store_true", help="rows/s counter only, no image")
    a = ap.parse_args()

    s = SpyScan(a.host, a.port, int(a.center), int(a.span), a.nbin, a.floor, a.rng)
    time.sleep(0.5)
    if s.dev:
        print("DeviceInfo: type=%d serial=%08X maxsps=%d decim=%d" % (s.dev[0], s.dev[1], s.dev[2], s.dev[4]))
    print("scan: center %.6f MHz, span %.3f MHz, %d bins, %d dBm + %d dB" %
          (a.center / 1e6, a.span / 1e6, a.nbin, a.floor, a.rng))

    if a.nogui:
        try:
            last = 0
            while s.alive:
                time.sleep(2.0)
                print("rows: %d  (%.1f rows/s)" % (s.n_lines, (s.n_lines - last) / 2.0))
                last = s.n_lines
        except KeyboardInterrupt:
            pass
        s.stop()
        return

    import matplotlib
    import matplotlib.pyplot as plt

    fig, (ax_sp, ax_wf) = plt.subplots(2, 1, figsize=(10, 7), height_ratios=[1, 3], sharex=True)
    fig.canvas.manager.set_window_title("FG23 scan — %s" % a.host)
    wf = np.full((a.rows, a.nbin), a.floor, dtype=np.float32)
    f0 = (a.center - a.span / 2) / 1e6
    f1 = (a.center + a.span / 2) / 1e6
    im = ax_wf.imshow(wf, aspect="auto", origin="upper", cmap="viridis",
                      extent=[f0, f1, a.rows, 0], vmin=a.floor, vmax=a.floor + a.rng)
    (ln,) = ax_sp.plot(np.linspace(f0, f1, a.nbin), wf[0], lw=1)
    ax_sp.set_ylim(a.floor, a.floor + a.rng)
    ax_sp.set_ylabel("dBm")
    ax_sp.grid(True, alpha=0.3)
    ax_wf.set_xlabel("MHz")
    ax_wf.set_ylabel("row (time ↓)")
    title = ax_sp.set_title("")
    state = {"rows": 0, "t0": time.time(), "n0": 0}

    def retune():
        s.center, s.span = int(s.center), int(s.span)
        s.apply()
        nf0 = (s.center - s.span / 2) / 1e6
        nf1 = (s.center + s.span / 2) / 1e6
        im.set_extent([nf0, nf1, a.rows, 0])
        ln.set_xdata(np.linspace(nf0, nf1, a.nbin))
        ax_sp.set_xlim(nf0, nf1)
        wf[:] = a.floor

    def on_key(ev):
        if ev.key == "q":
            plt.close(fig)
        elif ev.key == "left":
            s.center -= s.span / 4; retune()
        elif ev.key == "right":
            s.center += s.span / 4; retune()
        elif ev.key == "up":
            s.span = min(s.span * 2, 100e6); retune()
        elif ev.key == "down":
            s.span = max(s.span / 2, 100e3); retune()
        elif ev.key == "r":
            lo, hi = np.percentile(wf[wf > a.floor], [5, 99.5]) if np.any(wf > a.floor) else (a.floor, a.floor + a.rng)
            im.set_clim(lo, hi); ax_sp.set_ylim(lo - 5, hi + 5)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        got = 0
        while s.lines:
            row = s.lines.popleft()
            if row.size != a.nbin:
                continue
            wf[1:] = wf[:-1]
            wf[0] = row
            got += 1
        if got:
            im.set_data(wf)
            ln.set_ydata(wf[0])
            state["rows"] += got
        dt = time.time() - state["t0"]
        if dt >= 2.0:
            rate = (s.n_lines - state["n0"]) / dt
            state["t0"], state["n0"] = time.time(), s.n_lines
            title.set_text("center %.4f MHz  span %.2f MHz  %d bins   %.1f rows/s   total %d%s" %
                           (s.center / 1e6, s.span / 1e6, a.nbin, rate, s.n_lines,
                            "" if s.alive else "   [CONNECTION LOST]"))
        return im, ln, title

    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        s.stop()


if __name__ == "__main__":
    main()
