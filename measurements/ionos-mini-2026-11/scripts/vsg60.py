# SPDX-License-Identifier: MIT
"""Minimal Signal Hound VSG60 SCPI client (raw TCP, one command per line).

The VSG60 software must be running with SCPI enabled (default port 5124 in
the version used here). Only one SCPI connection is accepted at a time.
The output follows a command ~1-1.5 s later - wait before measuring.
Every set is read back so a lost command cannot silently corrupt a run.
"""
import os
import socket
import time

_ARB_FMT = {".csv": "CSV", ".bin": "BINSC", ".binsc": "BINSC",
            ".binfc": "BINFC", ".wav": "WAV"}


class VSG:
    def __init__(self, ip="127.0.0.1", port=5124, timeout=5.0):
        self.addr = (ip, port)
        self.timeout = timeout
        self.sock = socket.create_connection(self.addr, timeout=timeout)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def set(self, cmd, settle=0.1):
        self.sock.sendall((cmd + "\n").encode())
        if settle:
            time.sleep(settle)

    def query(self, cmd, timeout=None):
        self.sock.settimeout(timeout or self.timeout)
        try:
            self.sock.sendall((cmd + "\n").encode())
            return self.sock.recv(8192).decode(errors="replace").replace('"', "").strip()
        except Exception as e:
            return "TIMEOUT(%s)" % e
        finally:
            self.sock.settimeout(self.timeout)

    def idn(self):
        return self.query("*IDN?")

    def _num(self, q):
        try:
            return float(self.query(q).split(";")[0])
        except ValueError:
            return None

    def freq(self, hz, verify=True, tol_hz=1000.0):
        self.set("SOUR:FREQ %.0f" % hz)
        got = self._num("SOUR:FREQ?") if verify else hz
        return got is not None and abs(got - hz) <= tol_hz

    def power(self, dbm, verify=True, tol=0.6):
        self.set("SOUR:POW %.2f" % dbm)
        got = self._num("SOUR:POW?") if verify else dbm
        return got is not None and abs(got - dbm) <= tol

    def on(self):
        self.set("OUTP ON")

    def off(self):
        self.set("OUTP OFF")

    def modulation(self, on, verify=True):
        self.set(":OUTP:MOD %s" % ("ON" if on else "OFF"), settle=0.4)
        return (not verify) or self.query(":OUTP:MOD?").startswith("1" if on else "0")

    def load_arb(self, path, srate_hz=48000.0, settle=6.0, mod_on=True):
        """Load an I/Q file into the Arb generator and play it continuously."""
        fmt = _ARB_FMT.get(os.path.splitext(path)[1].lower())
        if fmt is None:
            return False, "unknown arb format: " + path
        self.set(":SOUR:RAD:ARB:WAV:UNLOAD", settle=0.5)
        self.set(':SOUR:RAD:ARB:WAV:LOAD:%s "%s"' % (fmt, path), settle=settle)
        n = self._num(":SOUR:RAD:ARB:WAV:LEN?")
        if not n:
            return False, "arb not loaded"
        n = int(n)
        for c in (":SOUR:RAD:ARB:SRAT %.0f" % srate_hz, ":SOUR:RAD:ARB:SAMP:OFFS 0",
                  ":SOUR:RAD:ARB:SAMP:COUN %d" % n, ":SOUR:RAD:ARB:SAMP:PER %d" % n,
                  ":SOUR:RAD:ARB:TRIG:TYPE CONT", ":SOUR:RAD:ARB:STAT ON"):
            self.set(c)
        if mod_on:
            self.modulation(True)
        return True, "%d samples @ %.0f S/s" % (n, srate_hz)
