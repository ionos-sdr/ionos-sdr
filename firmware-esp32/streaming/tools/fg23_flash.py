# SPDX-License-Identifier: MIT
"""Flash the FG23 on an ionos-sdr-mini through the ESP32-S3 (SWD from the ESP).

The ESP32 runs the normal streaming firmware (env ionos_mini); the image goes
over the diagnostic UART (the DevKit's CH343 port) with the FGFW command.

usage:  python fg23_flash.py <image.hex> --port COM4
        python fg23_flash.py --info --port COM4
"""
import argparse, binascii, sys, time
import serial


def load_hex(path):
    mem, base = {}, 0
    for ln in open(path):
        ln = ln.strip()
        if not ln.startswith(':'):
            continue
        b = bytes.fromhex(ln[1:])
        if sum(b) & 0xFF:
            raise ValueError('hex checksum: ' + ln)
        n, off, typ, data = b[0], (b[1] << 8) | b[2], b[3], b[4:4 + b[0]]
        if typ == 0:
            for i, x in enumerate(data):
                mem[base + off + i] = x
        elif typ == 2:
            base = int.from_bytes(data, 'big') << 4
        elif typ == 4:
            base = int.from_bytes(data, 'big') << 16
        elif typ == 1:
            break
    lo, hi = min(mem), max(mem) + 1
    img = bytearray(b'\xff' * (hi - lo))
    for a, x in mem.items():
        img[a - lo] = x
    return lo, bytes(img)


def open_port(port):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.2
    s.dtr = False          # do NOT reset the ESP32 (DevKit auto-reset circuit)
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()
    return s


def wait_line(s, prefixes, tmo):
    t0, buf = time.time(), b''
    while time.time() - t0 < tmo:
        buf += s.read(256)
        while b'\n' in buf:
            l, buf = buf.split(b'\n', 1)
            txt = l.decode(errors='replace').strip()
            if any(txt.startswith(p) for p in prefixes):
                return txt
            if txt.startswith(('FG23', 'GOT', 'halted', 'ERR', 'DONE')):
                print(txt, flush=True)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('hex', nargs='?')
    ap.add_argument('--port', default='COM4')
    ap.add_argument('--info', action='store_true')
    a = ap.parse_args()
    s = open_port(a.port)
    if a.info or not a.hex:
        s.write(b'\nFGINFO\n')
        print(wait_line(s, ['FG23 vectors', 'FG23 SWD FAIL'], 5))
        return
    addr, img = load_hex(a.hex)
    if addr < 0x08000000 or addr + len(img) > 0x08080000:
        sys.exit('image outside the FG23 main flash: %08X + %d' % (addr, len(img)))
    crc = binascii.crc32(img) & 0xFFFFFFFF
    print('image %d bytes @ %08X crc32 %08X' % (len(img), addr, crc))
    s.write(b'\nFGFW %08X %d %08X\n' % (addr, len(img), crc))
    r = wait_line(s, ['READY', 'ERR'], 5)
    if r != 'READY':
        sys.exit('no READY: %r' % r)
    t0 = time.time()
    for i in range(0, len(img), 2048):
        s.write(img[i:i + 2048]); s.flush()
        t1, got = time.time(), b''
        while b'#' not in got:
            got += s.read(1)
            if time.time() - t1 > 5:
                sys.exit('no chunk ack at %d' % i)
    print('sent in %.1f s, programming...' % (time.time() - t0), flush=True)
    r = wait_line(s, ['DONE', 'ERR'], 120)
    print(r)
    sys.exit(0 if r and r.startswith('DONE') else 1)


if __name__ == '__main__':
    main()
