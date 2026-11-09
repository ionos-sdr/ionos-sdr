import serial, time, sys
c = serial.Serial(); c.port = "COM4"; c.baudrate = 115200; c.timeout = 0.2; c.dtr = False; c.rts = False; c.open()
c.reset_input_buffer(); t0 = time.time(); o = b""
while time.time() - t0 < 3: o += c.read(4096)
for cmd in sys.argv[1:]:
    c.write((cmd + "\n").encode()); t0 = time.time()
    while time.time() - t0 < 2.5: o += c.read(4096)
print(o.decode("latin1")[-3500:])
c.close()
