#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# iq_bridge.py — ESP32-S3 nativ USB -> TCP, SDR++ szamara
#   Copyright (c) 2026 Zoltan Doczi HA7DCD
#
# Beolvassa az ESP32 nativ USB CDC portjarol a 1040 bajtos I/Q blokkokat,
# levagja a 16 bajtos fejlecet, es a nyers int16 mintakat kiszolgalja TCP-n.
#
#   antenna -> FG23 -> SPI -> ESP32-S3 -> USB -> [EZ] -> TCP -> SDR++
#
# HASZNALAT
#   python iq_bridge.py COM7
#   python iq_bridge.py /dev/ttyACM0 --port 8888
#
# SDR++ BEALLITAS
#   Source      : Network
#   Protocol    : TCP
#   Mode        : Client
#   Host        : 127.0.0.1
#   Port        : 8888
#   Sample type : Int16
#   Sample rate : a script kiirja a fejlecbol (i32 -> 12500)
#
# MIERT MEGY AT A FEJLEC AZ USB-N, ha ugyis levagjuk:
#   - a magicre barmikor ujra lehet szinkronizalni, tehat egy elveszett
#     bajt nem csusztatja el vegleg a folyamot;
#   - a sorszambol itt is latszik a vesztes, fuggetlenul attol, amit az
#     ESP mer. Ket fuggetlen pont ugyanarra a szamra.

import argparse
import socket
import sys
import threading
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("Hianyzik a pyserial:  pip install pyserial")

MAGIC = b"IQB2"          # 0x32425149 little-endian
HDR = 16
SAMPLES = 256
PAYLOAD = SAMPLES * 4
BLK = HDR + PAYLOAD      # 1040


class Stats:
    def __init__(self):
        self.blocks = 0
        self.lost = 0
        self.resync = 0
        self.tx = 0
        self.dropped = 0
        self.fs = 0
        self.decim = 0
        self.peak = 0
        self.t0 = time.time()

    def line(self):
        dt = max(time.time() - self.t0, 1e-6)
        return (f"{self.blocks/dt:6.1f} blokk/s  "
                f"{self.blocks*SAMPLES/dt:8.0f} sps  "
                f"csucs={self.peak:6d}  "
                f"VESZTETT={self.lost}  ujraszink={self.resync}  "
                f"TCP {self.tx/1024/dt:6.1f} kB/s eldobas={self.dropped}  "
                f"[fs_in={self.fs} decim={self.decim}]")

    def reset(self):
        self.blocks = self.lost = self.resync = self.tx = self.dropped = 0
        self.peak = 0
        self.t0 = time.time()


class TcpServer:
    """Egyetlen klienst szolgal ki. Ha nincs kliens, csendben eldobjuk az
    adatot — igy a soros olvasas soha nem torlodik meg."""

    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(1)
        self.client = None
        self.lock = threading.Lock()
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while True:
            c, addr = self.sock.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                if self.client:
                    try:
                        self.client.close()
                    except OSError:
                        pass
                self.client = c
            print(f"\n>>> SDR++ csatlakozott: {addr[0]}:{addr[1]}")

    def send(self, data):
        with self.lock:
            c = self.client
        if c is None:
            return 0
        try:
            c.sendall(data)
            return len(data)
        except OSError:
            print("\n<<< SDR++ lecsatlakozott")
            with self.lock:
                if self.client is c:
                    self.client = None
            return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("device", help="pl. COM7 vagy /dev/ttyACM0")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--baud", type=int, default=921600,
                    help="nativ USB CDC-nel ertelmetlen, de nem art")
    args = ap.parse_args()

    ser = serial.Serial(args.device, args.baud, timeout=1)
    srv = TcpServer(args.host, args.port)
    st = Stats()

    print(f"soros : {args.device}")
    print(f"TCP   : {args.host}:{args.port}  (SDR++ -> Network, TCP, Client)")
    print("SDR++ : Sample type = Int16, a mintavetelt lentrol olvasd le")
    print("varakozas az elso blokkra...\n")

    buf = bytearray()
    last_seq = None
    synced = False

    while True:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)

        while True:
            if not synced:
                # Ujraszinkronizalas: megkeressuk a magicet. Igy egy
                # elveszett bajt vagy egy odateveszett szoveges sor nem
                # csusztatja el veglegesen a folyamot.
                i = buf.find(MAGIC)
                if i < 0:
                    if len(buf) > 4 * BLK:
                        del buf[:-len(MAGIC)]
                    break
                if i:
                    del buf[:i]
                    st.resync += 1
                synced = True

            if len(buf) < BLK:
                break

            blk = bytes(buf[:BLK])
            if blk[:4] != MAGIC:
                synced = False
                continue
            del buf[:BLK]

            seq = int.from_bytes(blk[4:8], "little")
            nsamp = int.from_bytes(blk[8:10], "little")
            st.decim = int.from_bytes(blk[10:12], "little")
            st.fs = int.from_bytes(blk[12:16], "little")

            if last_seq is not None:
                d = (seq - last_seq) & 0xFFFFFFFF
                # A stream ujrainditasakor a FG23 nullazza a sorszamot, es a
                # kulonbseg egy hatalmas szam lesz. Az NEM vesztes — a regi
                # valtozat ezt tobb ezer elveszett blokknak szamolta, es az
                # elso statisztika-sor mindig ijesztoen nezett ki.
                if 1 < d < 100000:
                    st.lost += d - 1
            last_seq = seq
            st.blocks += 1

            payload = blk[HDR:HDR + nsamp * 4]

            # csucs (csak a statisztikahoz, minden 8. minta eleg)
            for k in range(0, len(payload), 32):
                v = int.from_bytes(payload[k:k + 2], "little", signed=True)
                if abs(v) > st.peak:
                    st.peak = abs(v)

            n = srv.send(payload)
            if n:
                st.tx += n
            else:
                st.dropped += 1

        if time.time() - st.t0 >= 2.0:
            out_sps = st.fs // st.decim if st.decim else 0
            print(f"\r{st.line()}  -> SDR++ sample rate: {out_sps}   ",
                  end="", flush=True)
            st.reset()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nvege")
