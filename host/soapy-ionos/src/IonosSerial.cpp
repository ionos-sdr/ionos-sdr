// SPDX-License-Identifier: MIT
//
// IonosSerial.cpp - see IonosSerial.hpp
//   Copyright (c) 2026 Zoltan Doczi HA7DCD

#include "IonosSerial.hpp"

#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <setupapi.h>
#include <initguid.h>
#include <devguid.h>
#include <cstdlib>
#else
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#endif

namespace ionos {

#ifdef _WIN32
// ============================== Windows =====================================

SerialPort::SerialPort() : _h(INVALID_HANDLE_VALUE), _rdEvent(nullptr), _wrEvent(nullptr), _rdTimeoutMs(-1) {}
SerialPort::~SerialPort() { close(); }
bool SerialPort::isOpen() const { return _h != INVALID_HANDLE_VALUE; }

bool SerialPort::open(const std::string &path, std::string &err, bool /*exclusive*/) {
    close();
    std::string dev = path;
    if (dev.rfind("\\\\.\\", 0) != 0) dev = "\\\\.\\" + dev;  // COM10+ need the device namespace
    HANDLE h = CreateFileA(dev.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                           FILE_FLAG_OVERLAPPED, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        const DWORD e = GetLastError();
        err = (e == ERROR_ACCESS_DENIED) ? "port busy (opened by another program)" : "CreateFile failed, error " + std::to_string(e);
        return false;
    }
    // usbser.sys queue: 4 MB (~5 s at 200 ksps) - the same size the Python
    // reference client asks for; 256 kB lost blocks on the first real-hardware run.
    SetupComm(h, 4 * 1024 * 1024, 4096);
    DCB dcb;
    std::memset(&dcb, 0, sizeof dcb);
    dcb.DCBlength = sizeof dcb;
    GetCommState(h, &dcb);
    dcb.BaudRate = 115200;  // ignored by CDC-ACM
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fNull = FALSE;
    dcb.fAbortOnError = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;   // DTR = 1: the device sends its hello
    dcb.fRtsControl = RTS_CONTROL_DISABLE;  // RTS = 0, never toggled
    if (!SetCommState(h, &dcb)) {
        err = "SetCommState failed, error " + std::to_string(GetLastError());
        CloseHandle(h);
        return false;
    }
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    _h = h;
    _rdEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
    _wrEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
    _rdTimeoutMs = -1;
    _path = path;
    return true;
}

void SerialPort::close() {
    if (_h != INVALID_HANDLE_VALUE) {
        CancelIo((HANDLE)_h);
        CloseHandle((HANDLE)_h);
        _h = INVALID_HANDLE_VALUE;
    }
    if (_rdEvent) { CloseHandle((HANDLE)_rdEvent); _rdEvent = nullptr; }
    if (_wrEvent) { CloseHandle((HANDLE)_wrEvent); _wrEvent = nullptr; }
}

long SerialPort::read(void *buf, size_t n, int timeoutMs) {
    if (!isOpen()) return -1;
    if (timeoutMs != _rdTimeoutMs) {
        // 0 / 0 / T: return when the buffer is full or after T ms, with whatever
        // arrived (pyserial's timeout= mode). The earlier MAXDWORD/MAXDWORD/T
        // returned after every USB transfer (~4 kB); with the reader's nap after
        // short reads (Windows sleep granularity ~15.6 ms) that drained only
        // ~300 kB/s of the 813 kB/s stream: 400 lost blocks in 30 s on hardware.
        COMMTIMEOUTS to;
        std::memset(&to, 0, sizeof to);
        to.ReadIntervalTimeout = 0;
        to.ReadTotalTimeoutMultiplier = 0;
        to.ReadTotalTimeoutConstant = (DWORD)(timeoutMs > 0 ? timeoutMs : 1);
        to.WriteTotalTimeoutConstant = 1000;
        SetCommTimeouts((HANDLE)_h, &to);
        _rdTimeoutMs = timeoutMs;
    }
    OVERLAPPED ov;
    std::memset(&ov, 0, sizeof ov);
    ov.hEvent = (HANDLE)_rdEvent;
    ResetEvent(ov.hEvent);
    DWORD got = 0;
    if (!ReadFile((HANDLE)_h, buf, (DWORD)n, &got, &ov)) {
        if (GetLastError() != ERROR_IO_PENDING) return -1;
        if (!GetOverlappedResult((HANDLE)_h, &ov, &got, TRUE)) return -1;
    }
    return (long)got;
}

bool SerialPort::write(const void *buf, size_t n) {
    if (!isOpen()) return false;
    OVERLAPPED ov;
    std::memset(&ov, 0, sizeof ov);
    ov.hEvent = (HANDLE)_wrEvent;
    ResetEvent(ov.hEvent);
    DWORD put = 0;
    if (!WriteFile((HANDLE)_h, buf, (DWORD)n, &put, &ov)) {
        if (GetLastError() != ERROR_IO_PENDING) return false;
        if (!GetOverlappedResult((HANDLE)_h, &ov, &put, TRUE)) return false;
    }
    return put == n;
}

// "USB\VID_303A&PID_4001\123456"
static void parseInstanceId(const std::string &id, UsbPortInfo &pi) {
    const size_t v = id.find("VID_"), p = id.find("PID_");
    if (v != std::string::npos) pi.vid = (uint16_t)std::strtoul(id.substr(v + 4, 4).c_str(), nullptr, 16);
    if (p != std::string::npos) pi.pid = (uint16_t)std::strtoul(id.substr(p + 4, 4).c_str(), nullptr, 16);
    const size_t s = id.rfind('\\');
    if (s != std::string::npos && id.find("&MI_") == std::string::npos) {
        const std::string ser = id.substr(s + 1);
        if (ser.find('&') == std::string::npos) pi.serial = ser;  // '&' = Windows-generated, not iSerial
    }
}

std::vector<UsbPortInfo> listUsbSerialPorts() {
    std::vector<UsbPortInfo> out;
    HDEVINFO set = SetupDiGetClassDevsA(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return out;
    SP_DEVINFO_DATA dd;
    dd.cbSize = sizeof dd;
    for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &dd); i++) {
        char id[512] = {0};
        if (!SetupDiGetDeviceInstanceIdA(set, &dd, id, sizeof id, nullptr)) continue;
        if (std::strncmp(id, "USB\\", 4) != 0) continue;
        UsbPortInfo pi;
        parseInstanceId(id, pi);
        HKEY key = SetupDiOpenDevRegKey(set, &dd, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (key == INVALID_HANDLE_VALUE) continue;
        char name[64] = {0};
        DWORD sz = sizeof name - 1, type = 0;
        const LONG r = RegQueryValueExA(key, "PortName", nullptr, &type, (LPBYTE)name, &sz);
        RegCloseKey(key);
        if (r != ERROR_SUCCESS || std::strncmp(name, "COM", 3) != 0) continue;
        pi.path = name;
        char friendly[256] = {0};
        if (SetupDiGetDeviceRegistryPropertyA(set, &dd, SPDRP_FRIENDLYNAME, nullptr, (PBYTE)friendly, sizeof friendly - 1, nullptr))
            pi.product = friendly;
        char mfg[256] = {0};
        if (SetupDiGetDeviceRegistryPropertyA(set, &dd, SPDRP_MFG, nullptr, (PBYTE)mfg, sizeof mfg - 1, nullptr))
            pi.manufacturer = mfg;
        out.push_back(pi);
    }
    SetupDiDestroyDeviceInfoList(set);
    return out;
}

#else
// ============================== POSIX =======================================

SerialPort::SerialPort() : _fd(-1) {}
SerialPort::~SerialPort() { close(); }
bool SerialPort::isOpen() const { return _fd >= 0; }

bool SerialPort::open(const std::string &path, std::string &err, bool exclusive) {
    close();
    // O_NONBLOCK: do not wait for carrier; I/O is driven by poll().
    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        err = "open " + path + ": " + std::strerror(errno);
        return false;
    }
    if (exclusive && ::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        err = "port busy (locked by another program): " + path;
        ::close(fd);
        return false;
    }
    struct termios tio;
    if (::tcgetattr(fd, &tio) == 0) {
        ::cfmakeraw(&tio);
        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        ::cfsetispeed(&tio, B115200);  // ignored by CDC-ACM
        ::cfsetospeed(&tio, B115200);
        ::tcsetattr(fd, TCSANOW, &tio);
    }
    // Linux raises DTR and RTS together on open (no reset: the S3 resets on
    // RTS=1 while DTR=0). Drop RTS, keep DTR. Ioctl failures are harmless
    // (ptys used by the mock have no modem lines).
    int bits = TIOCM_RTS;
    ::ioctl(fd, TIOCMBIC, &bits);
    bits = TIOCM_DTR;
    ::ioctl(fd, TIOCMBIS, &bits);
    // No tcflush(): the device sends its hello on the DTR edge, i.e. right
    // after open(), and a flush here could race it. Stale bytes are harmless
    // (the parser resyncs; replies are matched after each command is sent).
    _fd = fd;
    _path = path;
    return true;
}

void SerialPort::close() {
    if (_fd >= 0) {
        ::close(_fd);  // HUPCL drops DTR and RTS together: the device sees a disconnect
        _fd = -1;
    }
}

long SerialPort::read(void *buf, size_t n, int timeoutMs) {
    if (_fd < 0) return -1;
    struct pollfd pfd;
    pfd.fd = _fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    const int r = ::poll(&pfd, 1, timeoutMs);
    if (r < 0) return (errno == EINTR) ? 0 : -1;
    if (r == 0) return 0;
    if (pfd.revents & (POLLERR | POLLNVAL)) return -1;
    const ssize_t got = ::read(_fd, buf, n);
    if (got < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    if (got == 0 && (pfd.revents & POLLHUP)) return -1;
    return (long)got;
}

bool SerialPort::write(const void *buf, size_t n) {
    const uint8_t *p = (const uint8_t *)buf;
    int idle = 0;
    while (n && _fd >= 0) {
        const ssize_t w = ::write(_fd, p, n);
        if (w > 0) {
            p += w;
            n -= (size_t)w;
            continue;
        }
        if (w < 0 && errno != EAGAIN && errno != EINTR) return false;
        struct pollfd pfd = {_fd, POLLOUT, 0};
        if (::poll(&pfd, 1, 100) <= 0 && ++idle > 10) return false;
    }
    return n == 0;
}

static std::string readFileLine(const std::string &path) {
    std::ifstream f(path);
    std::string s;
    std::getline(f, s);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
    return s;
}

std::vector<UsbPortInfo> listUsbSerialPorts() {
    std::vector<UsbPortInfo> out;
#ifdef __APPLE__
    DIR *d = ::opendir("/dev");
    if (!d) return out;
    while (struct dirent *e = ::readdir(d)) {
        const std::string n = e->d_name;
        if (n.rfind("cu.usbmodem", 0) == 0) {
            UsbPortInfo pi;
            pi.path = "/dev/" + n;  // VID/PID unknown here: the hello decides
            out.push_back(pi);
        }
    }
    ::closedir(d);
#else
    DIR *d = ::opendir("/sys/class/tty");
    if (!d) return out;
    while (struct dirent *e = ::readdir(d)) {
        const std::string n = e->d_name;
        if (n.rfind("ttyACM", 0) != 0) continue;
        UsbPortInfo pi;
        pi.path = "/dev/" + n;
        // /sys/class/tty/ttyACM0/device -> .../<usbdev>/<usbdev>:1.0 (the interface)
        char real[PATH_MAX];
        const std::string link = "/sys/class/tty/" + n + "/device";
        if (::realpath(link.c_str(), real)) {
            std::string usb = real;
            const size_t s = usb.rfind('/');
            if (s != std::string::npos) usb.erase(s);
            pi.vid = (uint16_t)std::strtoul(readFileLine(usb + "/idVendor").c_str(), nullptr, 16);
            pi.pid = (uint16_t)std::strtoul(readFileLine(usb + "/idProduct").c_str(), nullptr, 16);
            pi.serial = readFileLine(usb + "/serial");
            pi.product = readFileLine(usb + "/product");
            pi.manufacturer = readFileLine(usb + "/manufacturer");
        }
        out.push_back(pi);
    }
    ::closedir(d);
#endif
    return out;
}

#endif

}  // namespace ionos
