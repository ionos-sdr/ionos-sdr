// SPDX-License-Identifier: MIT
//
// IonosRegistration.cpp - SoapySDR find/make for driver=ionos
//   Copyright (c) 2026 Zoltan Doczi HA7DCD
//
// Candidates: USB CDC ports with the Ionos VID/PID (TinyUSB 0x303A:0x4001, the
// older USB-Serial/JTAG path 0x303A:0x1001), on macOS every cu.usbmodem*, plus
// any ports listed in the IONOS_PORTS environment variable (separated by ',' or
// ';' - used for the mock device and unusual setups), or exactly port=<path>.
// Both PIDs are generic Espressif ones, so a candidate is only reported when it
// answers with the hello block (or, v1 firmware, streams IQB2 blocks).

#include "IonosDevice.hpp"

#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>

#include <algorithm>
#include <cstdlib>
#include <set>

namespace ionos {

static const uint16_t VID_ESPRESSIF = 0x303A;
static const uint16_t PID_TINYUSB_CDC = 0x4001;  // esp_tinyusb default PID, CDC only (IONOS_USB_TINYUSB=1)
static const uint16_t PID_USB_SERIAL_JTAG = 0x1001;
static const char *const IONOS_PRODUCT = "ionos-sdr-mini";
static const char *const NON_UNIQUE_SERIAL = "123456";  // esp_tinyusb Kconfig default iSerial

static std::vector<std::string> envPorts() {
    std::vector<std::string> out;
    const char *e = std::getenv("IONOS_PORTS");
    if (!e) return out;
    std::string s = e, cur;
    for (char c : s) {
        if (c == ',' || c == ';') {
            if (!cur.empty()) out.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

static std::string baseName(const std::string &p) {
    const size_t s = p.find_last_of("/\\");
    return s == std::string::npos ? p : p.substr(s + 1);
}

SoapySDR::KwargsList findIonos(const SoapySDR::Kwargs &args) {
    SoapySDR::KwargsList results;
    const std::vector<UsbPortInfo> ports = listUsbSerialPorts();

    std::vector<UsbPortInfo> cands;
    auto addPath = [&](const std::string &path) {
        for (const auto &c : cands)
            if (c.path == path) return;
        UsbPortInfo pi;
        pi.path = path;
        for (const auto &p : ports)
            if (p.path == path || baseName(p.path) == baseName(path)) pi = p, pi.path = path;
        cands.push_back(pi);
    };
    if (args.count("port")) {
        addPath(args.at("port"));
    } else {
        for (const auto &p : ports) {
            const bool idMatch = p.vid == VID_ESPRESSIF && (p.pid == PID_TINYUSB_CDC || p.pid == PID_USB_SERIAL_JTAG);
            if (idMatch || p.vid == 0) addPath(p.path);
        }
        for (const auto &p : envPorts()) addPath(p);
    }

    const bool doProbe = !(args.count("probe") && args.at("probe") == "0");
    for (const auto &c : cands) {
        Capabilities cap;
        bool sawIq = false, ok = false, busy = false;
        std::string err;
        if (doProbe) {
            ok = IonosDevice::probe(c.path, cap, sawIq, err);
            busy = !ok && err.find("busy") != std::string::npos;
        }
        const bool productMatch = c.product == IONOS_PRODUCT;
        if (!ok && !(busy && productMatch) && !(!doProbe && (productMatch || args.count("port")))) {
            SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyIonos: %s skipped (%s)", c.path.c_str(),
                           err.empty() ? "no hello, no I/Q" : err.c_str());
            continue;
        }

        SoapySDR::Kwargs r;
        r["driver"] = "ionos";
        r["port"] = c.path;
        std::string serial;
        if (cap.kv.count("serial")) serial = cap.kv.at("serial");
        else if (cap.kv.count("mac")) serial = cap.kv.at("mac");
        else if (!c.serial.empty() && c.serial != NON_UNIQUE_SERIAL) serial = c.serial;
        else serial = baseName(c.path);  // the firmware has no unique iSerial yet
        r["serial"] = serial;
        const std::string product = cap.kv.count("product") ? cap.kv.at("product") : (c.product.empty() ? "ionos-sdr" : c.product);
        r["product"] = product;
        if (cap.kv.count("board")) r["board"] = cap.kv.at("board");
        if (cap.kv.count("hw")) r["hw"] = cap.kv.at("hw");
        std::string label = "Ionos SDR " + product;
        if (cap.kv.count("board")) label += " [" + cap.kv.at("board") + "]";
        if (ok && !cap.fromDevice) label += " (v1 firmware)";
        if (busy) label += " (busy)";
        label += " :: " + c.path;
        r["label"] = label;

        if (args.count("serial") && args.at("serial") != serial) continue;
        results.push_back(r);
    }
    return results;
}

static SoapySDR::Device *makeIonos(const SoapySDR::Kwargs &args) { return new IonosDevice(args); }

static SoapySDR::Registry registerIonos("ionos", &findIonos, &makeIonos, SOAPY_SDR_ABI_VERSION);

}  // namespace ionos
