// SPDX-License-Identifier: MIT
//
// IonosSerial.hpp - minimal portable CDC-ACM serial port + USB port listing
//   Copyright (c) 2026 Zoltan Doczi HA7DCD
//
// Linux / macOS: termios. Windows: overlapped Win32 COM I/O, SetupAPI listing.
// The port is always opened with DTR asserted and RTS de-asserted and RTS is
// never touched again: the ESP32-S3 USB-Serial/JTAG resets the chip when RTS
// is asserted while DTR is low (docs/ionos-usb-protocol.md, 7b host rule).

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ionos {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();
    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    // exclusive: take an advisory lock (Linux/macOS flock) so two processes
    // cannot split one stream; on Windows COM ports are exclusive anyway.
    bool open(const std::string &path, std::string &err, bool exclusive = true);
    void close();
    bool isOpen() const;

    // Bytes read (0 = timeout), -1 = port error (unplugged).
    long read(void *buf, size_t n, int timeoutMs);
    // Writes everything or fails.
    bool write(const void *buf, size_t n);

    const std::string &path() const { return _path; }

private:
    std::string _path;
#ifdef _WIN32
    void *_h;          // HANDLE
    void *_rdEvent;    // HANDLE
    void *_wrEvent;    // HANDLE
    int _rdTimeoutMs;
#else
    int _fd;
#endif
};

struct UsbPortInfo {
    std::string path;          // /dev/ttyACM0, \\.\COM7, /dev/cu.usbmodem1101
    uint16_t vid = 0, pid = 0; // 0 = unknown (macOS listing)
    std::string serial;        // USB iSerialNumber
    std::string product;       // USB iProduct (Linux) or friendly name (Windows)
    std::string manufacturer;
};

// CDC-ACM style ports of the system with whatever USB identity is cheaply
// available. No port is opened.
std::vector<UsbPortInfo> listUsbSerialPorts();

}  // namespace ionos
