// SPDX-License-Identifier: MIT
//
// IonosDevice.cpp - see IonosDevice.hpp
//   Copyright (c) 2026 Zoltan Doczi HA7DCD
//
// Threads: the reader thread owns the serial RX side and the frame parser,
// tracks the IQB2 sequence (zero-fill on gaps), feeds the sample ring and
// collects MSG1 replies. API calls write command lines and wait for replies.

#include "IonosDevice.hpp"

#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace ionos {

SoapySDR::KwargsList findIonos(const SoapySDR::Kwargs &args);  // IonosRegistration.cpp

static const char *const REJECT_LETTERS = "cwpxbzydkngvM";  // spec 4.1: TX / bench / diagnostics
static const double SHIFT_DB = 6.0206;                      // one bit of digital shift

static double nowS() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

static bool argBool(const SoapySDR::Kwargs &a, const std::string &k, bool def) {
    auto it = a.find(k);
    if (it == a.end()) return def;
    const std::string &v = it->second;
    return !(v == "0" || v == "false" || v == "no" || v == "off");
}

static std::string trimText(const char *t, size_t n) {
    std::string s(t, n);
    while (!s.empty() && (s.back() == '\0' || s.back() == '\n' || s.back() == '\r')) s.pop_back();
    return s;
}

static std::string oneLine(std::string s) {
    std::replace(s.begin(), s.end(), '\n', ' ');
    return s;
}

// ============================ Capabilities ===================================

void Capabilities::fromKeyValues(const std::map<std::string, std::string> &m) {
    fromDevice = true;
    for (const auto &p : m) kv[p.first] = p.second;
    auto num = [&](const char *k, double def) {
        auto it = kv.find(k);
        if (it == kv.end()) return def;
        try { return std::stod(it->second); } catch (...) { return def; }
    };
    fsIn = num("fs_in_hz", fsIn);
    freqMin = num("freq_min_hz", freqMin);
    freqMax = num("freq_max_hz", freqMax);
    gainShiftMax = (int)num("gain_shift_max", gainShiftMax);
    spectrum = num("spectrum", 0) != 0;
    auto it = kv.find("decim_list");
    if (it != kv.end()) {
        std::vector<int> d = parseIntList(it->second);
        if (!d.empty()) decims = d;
    }
}

// ============================ construction ===================================

IonosDevice::IonosDevice(const SoapySDR::Kwargs &args) : _args(args) {
    std::string path;
    auto it = args.find("port");
    if (it != args.end()) {
        path = it->second;
    } else {
        const SoapySDR::KwargsList found = findIonos(args);
        if (found.empty()) throw std::runtime_error("SoapyIonos: no Ionos SDR found");
        path = found.front().at("port");
    }

    _zeroFill = argBool(args, "zero_fill", true);
    _reportGaps = argBool(args, "report_gaps", true);
    if (args.count("max_fill_blocks")) _maxFillBlocks = (uint32_t)std::stoul(args.at("max_fill_blocks"));

    size_t ringSamples = 1u << 20;  // ~5 s at 200 ksps
    if (args.count("ring_samples")) ringSamples = (size_t)std::stoul(args.at("ring_samples"));
    _ringN = 1;
    while (_ringN < ringSamples) _ringN <<= 1;
    _ring.assign(2 * _ringN, 0);

    std::string err;
    if (!_port.open(path, err, true)) throw std::runtime_error("SoapyIonos: " + err);
    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyIonos: opened %s (DTR=1, RTS=0)", path.c_str());

    _running = true;
    _reader = std::thread(&IonosDevice::readerLoop, this);

    if (!handshake(0.15, 1.5)) {
        if (_iqSeen == 0) {
            _running = false;
            _reader.join();
            _port.close();
            throw std::runtime_error("SoapyIonos: no hello and no I/Q on " + path + " - not an Ionos SDR?");
        }
        SoapySDR::logf(SOAPY_SDR_WARNING,
                       "SoapyIonos: no hello on %s but I/Q flows: v1 firmware, fixed 50 ksps / 430-440 MHz, F/i only",
                       path.c_str());
    } else {
        const std::string ver = command("VER?");
        if (ver.compare(0, 2, "OK") == 0) {
            std::lock_guard<std::mutex> lk(_infoMutex);
            _ver = parseKeyValues(ver);
        }
        const std::string mode = args.count("mode") ? args.at("mode") : "usb";
        if (mode == "usb" || mode == "USB") {
            const std::string r = command("MODE USB");
            _sentModeUsb = r.compare(0, 2, "OK") == 0;
            if (!_sentModeUsb) SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: MODE USB -> '%s'", oneLine(r).c_str());
        }
        if (args.count("stat")) command("STAT " + args.at("stat"));
    }

    std::lock_guard<std::mutex> lk(_infoMutex);
    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyIonos: %s board=%s fs_in=%.0f decim_list=%s freq=%.0f..%.0f Hz",
                   _cap.fromDevice ? (_cap.kv.count("product") ? _cap.kv.at("product").c_str() : "ionos") : "v1",
                   _cap.kv.count("board") ? _cap.kv.at("board").c_str() : "?", _cap.fsIn,
                   _cap.kv.count("decim_list") ? _cap.kv.at("decim_list").c_str() : "8", _cap.freqMin, _cap.freqMax);
}

IonosDevice::~IonosDevice() {
    _active = false;
    if (_sentModeUsb && argBool(_args, "restore_mode", true) && !_portError) command("MODE STANDALONE", 0.5);
    _running = false;
    if (_reader.joinable()) _reader.join();
    _port.close();
}

bool IonosDevice::handshake(double helloWaitS, double capWaitS) {
    // Spec section 6 rule 5: wait for the hello (sent on the DTR edge), else ask
    // with CAP?. The wait is short: a close/reopen faster than the device's
    // main loop (e.g. find() then make()) produces no edge and no hello.
    double t0 = nowS();
    while (!_haveCap && nowS() - t0 < helloWaitS && !_portError)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (_haveCap) return true;
    sendRaw("CAP?");
    t0 = nowS();
    while (!_haveCap && nowS() - t0 < capWaitS && !_portError)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return _haveCap;
}

// Static probe for find(): no reader thread, just read until a hello shows up.
bool IonosDevice::probe(const std::string &path, Capabilities &cap, bool &sawIq, std::string &err) {
    struct ProbeSink : FrameSink {
        bool hello = false, iq = false;
        std::map<std::string, std::string> kv;
        void onIQ(const BlockHeader &, const uint8_t *) override { iq = true; }
        void onSpectrum(const BlockHeader &, const uint8_t *) override {}
        void onMessage(const BlockHeader &h, const char *t, size_t n) override {
            if (h.decim == MSG_HELLO) {
                kv = parseKeyValues(trimText(t, n));
                hello = kv.count("ionos") != 0;
            }
        }
    } sink;
    SerialPort port;
    if (!port.open(path, err, true)) return false;
    FrameParser parser(64 * 1024);
    bool askedCap = false;
    const double t0 = nowS();
    while (!sink.hello) {
        const double dt = nowS() - t0;
        if (dt > 1.2) break;
        if (!askedCap && dt > 0.15) {
            const char *l = "CAP?\n";
            port.write(l, 5);
            askedCap = true;
        }
        const long n = port.read(parser.prepare(16384), 16384, 50);
        if (n < 0) { err = "read error"; break; }
        if (n > 0) parser.commit((size_t)n, sink);
    }
    port.close();
    sawIq = sink.iq;
    if (sink.hello) cap.fromKeyValues(sink.kv);
    return sink.hello || sink.iq;
}

// ============================ reader thread ==================================

void IonosDevice::readerLoop() {
    const size_t chunk = 64 * 1024;
    while (_running) {
        uint8_t *dst = _parser.prepare(chunk);
        const long n = _port.read(dst, chunk, 50);
        if (n < 0) {
            if (!_portError) SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyIonos: port read error (device unplugged?)");
            _portError = true;
            _dataCv.notify_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (n == 0) continue;
        const uint64_t before = _wpos.load(std::memory_order_relaxed);
        _parser.commit((size_t)n, *this);
        if (_active) {  // like the other counters: frozen while deactivated
            _parserResyncs.store(_parser.resyncs, std::memory_order_relaxed);
            _parserTrunc.store(_parser.truncated, std::memory_order_relaxed);
        }
        if (_wpos.load(std::memory_order_relaxed) != before) {
            std::lock_guard<std::mutex> lk(_dataMutex);
            _dataCv.notify_one();
        }
        // CDC drivers hand data over per USB transfer (often 64-512 B); after a
        // short read let it accumulate so each read moves kilobytes, not packets.
        // The OS buffers (64 kB tty / 256 kB usbser) cover the 1-16 ms nap.
        if (n < 8192) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool IonosDevice::logAllowed() {
    const double t = nowS();
    if (t - _logWindowStart > 1.0) {
        if (_logSuppressed)
            SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: %d more loss events suppressed", _logSuppressed);
        _logWindowStart = t;
        _logCount = 0;
        _logSuppressed = 0;
    }
    if (++_logCount <= 5) return true;
    _logSuppressed++;
    return false;
}

void IonosDevice::onIQ(const BlockHeader &h, const uint8_t *payload) {
    _iqSeen++;
    _rssi = h.rssiDbm();
    const bool active = _active.load();
    bool first = false;
    if (active && _resetStats.exchange(false)) {
        _lostBlocks = 0; _gapEvents = 0; _restarts = 0; _hostOverflows = 0; _blocksIn = 0;
        _spcBlocks = 0; _msgBlocks = 0; _filledSamples = 0;
        _resyncBase = _parser.resyncs;
        _truncBase = _parser.truncated;
        first = true;
    }

    if ((int)h.decim != _lastHdrDecim) {
        if (_lastHdrDecim && active) {
            _restarts++;
            SoapySDR::logf(SOAPY_SDR_INFO, "SoapyIonos: stream rate changed, R %d -> %d", _lastHdrDecim, (int)h.decim);
        }
        _lastHdrDecim = h.decim;
        _curDecim = h.decim;
        _haveLastSeq = false;
    }

    if (_haveLastSeq && !first && active) {
        const uint32_t expect = _lastSeq + 1;
        if (h.seq != expect) {
            const uint32_t gap = h.seq - expect;
            if (h.seq < expect) {
                // seq restarts at 0 when the FG23 restarts the stream (F / i / R): not a loss
                _restarts++;
                if (logAllowed())
                    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyIonos: stream restart (seq %u -> %u)", _lastSeq, h.seq);
            } else {
                _lostBlocks += gap;
                _gapEvents++;
                if (_reportGaps) _overflowPending = true;
                const bool fill = _zeroFill && gap <= _maxFillBlocks;
                if (fill) pushZeros((size_t)gap * _lastNsamp);
                if (logAllowed())
                    SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: lost %u block(s) (seq %u..%u)%s", gap, expect,
                                   h.seq - 1, fill ? ", zero-filled" : ", NOT filled");
            }
        }
    }
    _haveLastSeq = true;
    _lastSeq = h.seq;
    _lastNsamp = h.nsamp;

    if (active) {
        if (first) _firstSeq = h.seq;
        pushSamples(payload, h.nsamp);
        _blocksIn++;
        _lastSeqPub = h.seq;
    }
}

void IonosDevice::onSpectrum(const BlockHeader &, const uint8_t *) {
    // SPC1 (scan mode) does not belong in the I/Q path. TODO: expose scans.
    if (_active) _spcBlocks++;
}

void IonosDevice::onMessage(const BlockHeader &h, const char *t, size_t n) {
    const std::string text = trimText(t, n);
    if (_active) _msgBlocks++;
    switch (h.decim) {
    case MSG_RESPONSE: {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyIonos: MSG1 response: %s", oneLine(text).c_str());
        std::lock_guard<std::mutex> lk(_respMutex);
        _responses.push_back(text);
        if (_responses.size() > 16) _responses.pop_front();
        _respCv.notify_all();
        break;
    }
    case MSG_HELLO: {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyIonos: MSG1 hello: %s", oneLine(text).c_str());
        const auto kv = parseKeyValues(text);
        if (kv.count("ionos")) {
            std::lock_guard<std::mutex> lk(_infoMutex);
            _cap.fromKeyValues(kv);
            _haveCap = true;
        }
        break;
    }
    case MSG_STATUS: {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyIonos: MSG1 status: %s", oneLine(text).c_str());
        std::lock_guard<std::mutex> lk(_infoMutex);
        _status = parseKeyValues(text);
        break;
    }
    case MSG_ERROR:
    default: {
        SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: device: %s", oneLine(text).c_str());
        std::lock_guard<std::mutex> lk(_infoMutex);
        _lastError = text;
        break;
    }
    }
}

// ============================ sample ring ====================================

// The payload is little-endian int16 I/Q; on a little-endian host it is
// copied as is (all supported targets are LE).
void IonosDevice::pushSamples(const uint8_t *iq, size_t n) {
    const uint64_t w = _wpos.load(std::memory_order_relaxed);
    const uint64_t r = _rpos.load(std::memory_order_acquire);
    if (w - r + n > _ringN) {
        _hostOverflows++;
        _overflowPending = true;
        return;
    }
    const size_t idx = (size_t)(w & (_ringN - 1));
    const size_t first = std::min(n, _ringN - idx);
    std::memcpy(&_ring[2 * idx], iq, first * 4);
    if (n > first) std::memcpy(&_ring[0], iq + first * 4, (n - first) * 4);
    _wpos.store(w + n, std::memory_order_release);
}

void IonosDevice::pushZeros(size_t n) {
    const uint64_t w = _wpos.load(std::memory_order_relaxed);
    const uint64_t r = _rpos.load(std::memory_order_acquire);
    if (w - r + n > _ringN) {
        _hostOverflows++;
        _overflowPending = true;
        return;
    }
    const size_t idx = (size_t)(w & (_ringN - 1));
    const size_t first = std::min(n, _ringN - idx);
    std::memset(&_ring[2 * idx], 0, first * 4);
    if (n > first) std::memset(&_ring[0], 0, (n - first) * 4);
    _filledSamples += n;
    _wpos.store(w + n, std::memory_order_release);
}

// ============================ commands =======================================

void IonosDevice::sendRaw(const std::string &line) {
    const std::string l = line + "\n";
    if (!_port.write(l.data(), l.size()))
        SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyIonos: write failed: %s", line.c_str());
}

std::string IonosDevice::command(const std::string &line, double timeoutS) {
    std::lock_guard<std::mutex> cmdLock(_cmdMutex);
    {
        std::lock_guard<std::mutex> lk(_respMutex);
        _responses.clear();
    }
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyIonos: -> %s", line.c_str());
    sendRaw(line);
    std::unique_lock<std::mutex> lk(_respMutex);
    const bool got = _respCv.wait_for(lk, std::chrono::microseconds((long long)(timeoutS * 1e6)),
                                      [&] { return !_responses.empty(); });
    if (!got) {
        SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: no reply to '%s'", line.c_str());
        return "";
    }
    std::string r = _responses.front();
    _responses.pop_front();
    lk.unlock();
    std::lock_guard<std::mutex> il(_infoMutex);
    _lastResponse = r;
    return r;
}

// ============================ identification =================================

std::string IonosDevice::getDriverKey() const { return "ionos"; }

std::string IonosDevice::getHardwareKey() const {
    std::lock_guard<std::mutex> lk(_infoMutex);
    auto it = _cap.kv.find("product");
    return it != _cap.kv.end() ? it->second : "ionos-sdr";
}

SoapySDR::Kwargs IonosDevice::getHardwareInfo() const {
    std::lock_guard<std::mutex> lk(_infoMutex);
    SoapySDR::Kwargs info;
    for (const auto &p : _cap.kv) info[p.first] = p.second;
    for (const auto &p : _ver) info[p.first] = p.second;
    info["port"] = _port.path();
    info["protocol"] = _cap.fromDevice ? "v2" : "v1 (no hello)";
    return info;
}

size_t IonosDevice::getNumChannels(const int direction) const { return direction == SOAPY_SDR_RX ? 1 : 0; }

// ============================ stream =========================================

std::vector<std::string> IonosDevice::getStreamFormats(const int, const size_t) const {
    return {SOAPY_SDR_CS16, SOAPY_SDR_CF32, SOAPY_SDR_CS8};
}

std::string IonosDevice::getNativeStreamFormat(const int, const size_t, double &fullScale) const {
    fullScale = 32768.0;
    return SOAPY_SDR_CS16;
}

SoapySDR::Stream *IonosDevice::setupStream(const int direction, const std::string &format,
                                          const std::vector<size_t> &channels, const SoapySDR::Kwargs &) {
    if (direction != SOAPY_SDR_RX) throw std::runtime_error("SoapyIonos: RX only");
    if (channels.size() > 1 || (channels.size() == 1 && channels[0] != 0))
        throw std::runtime_error("SoapyIonos: one channel (0)");
    if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32 && format != SOAPY_SDR_CS8)
        throw std::runtime_error("SoapyIonos: format " + format + " not supported (CS16, CF32, CS8)");
    if (_streamSetup.exchange(true)) throw std::runtime_error("SoapyIonos: stream already set up");
    _format = format;
    return reinterpret_cast<SoapySDR::Stream *>(this);
}

void IonosDevice::closeStream(SoapySDR::Stream *) {
    _active = false;
    _streamSetup = false;
}

size_t IonosDevice::getStreamMTU(SoapySDR::Stream *) const { return 8 * IQ_MAX_SAMPLES; }

int IonosDevice::activateStream(SoapySDR::Stream *, const int flags, const long long, const size_t) {
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    // The device streams continuously (the FG23 autostarts; the protocol has no
    // stateless start/stop, 'i' toggles). Activation gates the host side.
    _overflowPending = false;
    _rpos.store(_wpos.load());
    _resetStats = true;
    _active = true;
    return 0;
}

int IonosDevice::deactivateStream(SoapySDR::Stream *, const int flags, const long long) {
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    _active = false;
    return 0;
}

int IonosDevice::readStream(SoapySDR::Stream *, void *const *buffs, const size_t numElems, int &flags,
                            long long &timeNs, const long timeoutUs) {
    flags = 0;
    timeNs = 0;
    if (!_active) return SOAPY_SDR_STREAM_ERROR;
    if (_overflowPending.exchange(false)) return SOAPY_SDR_OVERFLOW;

    uint64_t r = _rpos.load(std::memory_order_relaxed);
    uint64_t avail = _wpos.load(std::memory_order_acquire) - r;
    if (avail == 0) {
        std::unique_lock<std::mutex> lk(_dataMutex);
        _dataCv.wait_for(lk, std::chrono::microseconds(timeoutUs),
                         [&] { return _wpos.load(std::memory_order_acquire) != r || _portError.load(); });
        avail = _wpos.load(std::memory_order_acquire) - r;
        if (avail == 0) return _portError ? SOAPY_SDR_STREAM_ERROR : SOAPY_SDR_TIMEOUT;
    }
    const size_t n = (size_t)std::min<uint64_t>(avail, numElems);
    size_t done = 0;
    while (done < n) {
        const size_t idx = (size_t)((r + done) & (_ringN - 1));
        const size_t seg = std::min(n - done, _ringN - idx);
        const int16_t *src = &_ring[2 * idx];
        if (_format == SOAPY_SDR_CS16) {
            std::memcpy((int16_t *)buffs[0] + 2 * done, src, seg * 4);
        } else if (_format == SOAPY_SDR_CF32) {
            float *dst = (float *)buffs[0] + 2 * done;
            const float k = 1.0f / 32768.0f;
            for (size_t i = 0; i < 2 * seg; i++) dst[i] = src[i] * k;
        } else {  // CS8 = int16 >> 8 (spec section 6 rule 3)
            int8_t *dst = (int8_t *)buffs[0] + 2 * done;
            for (size_t i = 0; i < 2 * seg; i++) dst[i] = (int8_t)(src[i] >> 8);
        }
        done += seg;
    }
    _rpos.store(r + n, std::memory_order_release);
    return (int)n;
}

// ============================ antenna / gain =================================

std::vector<std::string> IonosDevice::listAntennas(const int, const size_t) const { return {"RX"}; }
std::string IonosDevice::getAntenna(const int, const size_t) const { return "RX"; }

std::vector<std::string> IonosDevice::listGains(const int, const size_t) const { return {"DIGITAL"}; }

SoapySDR::Range IonosDevice::getGainRange(const int, const size_t, const std::string &) const {
    std::lock_guard<std::mutex> lk(_infoMutex);
    return SoapySDR::Range(0.0, _cap.gainShiftMax * SHIFT_DB, SHIFT_DB);
}

void IonosDevice::setGain(const int, const size_t, const std::string &, const double value) {
    int maxShift;
    bool v2;
    {
        std::lock_guard<std::mutex> lk(_infoMutex);
        maxShift = _cap.gainShiftMax;
        v2 = _cap.fromDevice;
    }
    const int shift = std::max(0, std::min(maxShift, (int)std::lround(value / SHIFT_DB)));
    if (maxShift <= 0 || !v2) {
        _gainShift = 0;
        return;  // gain_shift_max=0: nothing to set (the firmware answers G with ERR today)
    }
    const std::string r = command("G" + std::to_string(shift));
    if (r.compare(0, 2, "OK") == 0) _gainShift = shift;
    else SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: G%d -> '%s'", shift, oneLine(r).c_str());
}

double IonosDevice::getGain(const int, const size_t, const std::string &) const { return _gainShift * SHIFT_DB; }

// ============================ frequency ======================================

void IonosDevice::setFrequency(const int, const size_t, const std::string &, const double frequency,
                               const SoapySDR::Kwargs &) {
    bool v2;
    double fmin, fmax;
    {
        std::lock_guard<std::mutex> lk(_infoMutex);
        v2 = _cap.fromDevice;
        fmin = _cap.freqMin;
        fmax = _cap.freqMax;
    }
    if (frequency < fmin || frequency > fmax)
        SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: %.0f Hz is outside the board range %.0f..%.0f Hz", frequency,
                       fmin, fmax);
    const long long hz = std::llround(frequency);
    if (!v2) {
        sendRaw("F" + std::to_string((hz + 500) / 1000));  // v1: kHz only
        _freqHz = (double)(((hz + 500) / 1000) * 1000);
        return;
    }
    const std::string r = command("FH" + std::to_string(hz));
    if (r.compare(0, 2, "OK") != 0) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyIonos: FH%lld -> '%s'", hz, oneLine(r).c_str());
        return;
    }
    const auto kv = parseKeyValues(r);
    auto it = kv.find("f");
    _freqHz = (it != kv.end()) ? std::atof(it->second.c_str()) : (double)hz;
}

double IonosDevice::getFrequency(const int, const size_t, const std::string &) const { return _freqHz; }

std::vector<std::string> IonosDevice::listFrequencies(const int, const size_t) const { return {"RF"}; }

SoapySDR::RangeList IonosDevice::getFrequencyRange(const int, const size_t, const std::string &) const {
    std::lock_guard<std::mutex> lk(_infoMutex);
    return {SoapySDR::Range(_cap.freqMin, _cap.freqMax)};
}

// ============================ sample rate ====================================

int IonosDevice::pickDecim(double rate) const {
    std::lock_guard<std::mutex> lk(_infoMutex);
    int best = _cap.decims.front();
    for (int d : _cap.decims)
        if (std::fabs(_cap.fsIn / d - rate) < std::fabs(_cap.fsIn / best - rate)) best = d;
    return best;
}

void IonosDevice::setSampleRate(const int, const size_t, const double rate) {
    const int d = pickDecim(rate);
    double fsIn;
    bool v2;
    {
        std::lock_guard<std::mutex> lk(_infoMutex);
        fsIn = _cap.fsIn;
        v2 = _cap.fromDevice;
    }
    if (std::fabs(fsIn / d - rate) > 1.0)
        SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: %.0f sps not available, using %.1f sps (R=%d)", rate, fsIn / d, d);
    _reqDecim = d;
    if (!v2) return;
    if (_curDecim == d) return;
    const std::string r = command("R" + std::to_string(std::llround(fsIn / d)));
    if (r.compare(0, 2, "OK") != 0) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyIonos: R -> '%s'", oneLine(r).c_str());
        return;
    }
    // The ESP stops and restarts the FG23 stream; wait until the headers carry the new R.
    const double t0 = nowS();
    while (_curDecim != d && nowS() - t0 < 3.0 && !_portError)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (_curDecim != d)
        SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: stream still at R=%d after R request %d", (int)_curDecim, d);
}

double IonosDevice::getSampleRate(const int, const size_t) const {
    std::lock_guard<std::mutex> lk(_infoMutex);
    int d = _curDecim ? (int)_curDecim : (_reqDecim ? (int)_reqDecim : _cap.decims.front());
    return _cap.fsIn / d;
}

std::vector<double> IonosDevice::listSampleRates(const int, const size_t) const {
    std::lock_guard<std::mutex> lk(_infoMutex);
    std::vector<double> r;
    for (int d : _cap.decims) r.push_back(_cap.fsIn / d);
    std::sort(r.begin(), r.end());
    return r;
}

SoapySDR::RangeList IonosDevice::getSampleRateRange(const int dir, const size_t ch) const {
    SoapySDR::RangeList out;
    for (double r : listSampleRates(dir, ch)) out.push_back(SoapySDR::Range(r, r));
    return out;
}

double IonosDevice::getBandwidth(const int dir, const size_t ch) const { return getSampleRate(dir, ch); }
std::vector<double> IonosDevice::listBandwidths(const int dir, const size_t ch) const { return listSampleRates(dir, ch); }

// ============================ sensors ========================================

static const char *const SENSORS[][2] = {
    {"lost_blocks", "IQB2 blocks missing in the seq (zero-filled), since activateStream"},
    {"gap_events", "number of seq gaps, since activateStream"},
    {"filled_samples", "zero samples inserted for lost blocks"},
    {"truncated_blocks", "blocks cut short on the link (resync on the next magic), since activateStream"},
    {"resyncs", "framing resynchronisations, since activateStream"},
    {"stream_restarts", "device stream restarts (seq reset / rate change), since activateStream"},
    {"host_overflows", "blocks dropped because the application read too slowly"},
    {"blocks", "IQB2 blocks received, since activateStream"},
    {"first_seq", "seq of the first block after activateStream"},
    {"last_seq", "seq of the latest block"},
    {"spc_blocks", "SPC1 (scan) blocks skipped"},
    {"msg_blocks", "MSG1 blocks received"},
    {"rssi", "RSSI from the IQB2 header [dBm]"},
    {"dev_lost", "device-side lost blocks (STAT lost=)"},
    {"dev_usb_drop", "device-side USB drops (STAT usb_drop=)"},
    {"dev_blk_s", "device block rate (STAT blk_s=)"},
    {"dev_mode", "device mode (STAT mode=)"},
    {"link", "ok / error"},
};

std::vector<std::string> IonosDevice::listSensors() const {
    std::vector<std::string> s;
    for (const auto &e : SENSORS) s.push_back(e[0]);
    return s;
}

SoapySDR::ArgInfo IonosDevice::getSensorInfo(const std::string &key) const {
    SoapySDR::ArgInfo a;
    a.key = key;
    a.name = key;
    a.type = SoapySDR::ArgInfo::INT;
    for (const auto &e : SENSORS)
        if (key == e[0]) a.description = e[1];
    if (key == "dev_blk_s") a.type = SoapySDR::ArgInfo::FLOAT;
    if (key == "dev_mode" || key == "link") a.type = SoapySDR::ArgInfo::STRING;
    if (key == "rssi") a.units = "dBm";
    return a;
}

std::string IonosDevice::readSensor(const std::string &key) const {
    if (key == "lost_blocks") return std::to_string(_lostBlocks.load());
    if (key == "gap_events") return std::to_string(_gapEvents.load());
    if (key == "filled_samples") return std::to_string(_filledSamples.load());
    if (key == "truncated_blocks") { const uint64_t a = _parserTrunc.load(), b = _truncBase.load(); return std::to_string(a > b ? a - b : 0); }
    if (key == "resyncs") { const uint64_t a = _parserResyncs.load(), b = _resyncBase.load(); return std::to_string(a > b ? a - b : 0); }
    if (key == "stream_restarts") return std::to_string(_restarts.load());
    if (key == "host_overflows") return std::to_string(_hostOverflows.load());
    if (key == "blocks") return std::to_string(_blocksIn.load());
    if (key == "first_seq") return std::to_string(_firstSeq.load());
    if (key == "last_seq") return std::to_string(_lastSeqPub.load());
    if (key == "spc_blocks") return std::to_string(_spcBlocks.load());
    if (key == "msg_blocks") return std::to_string(_msgBlocks.load());
    if (key == "rssi") return std::to_string(_rssi.load());
    if (key == "link") return _portError ? "error" : "ok";
    std::lock_guard<std::mutex> lk(_infoMutex);
    auto st = [&](const char *k) {
        auto it = _status.find(k);
        return it == _status.end() ? std::string() : it->second;
    };
    if (key == "dev_lost") return st("lost");
    if (key == "dev_usb_drop") return st("usb_drop");
    if (key == "dev_blk_s") return st("blk_s");
    if (key == "dev_mode") return st("mode");
    throw std::runtime_error("SoapyIonos: unknown sensor " + key);
}

std::vector<std::string> IonosDevice::listSensors(const int, const size_t) const { return {"rssi"}; }
SoapySDR::ArgInfo IonosDevice::getSensorInfo(const int, const size_t, const std::string &key) const {
    return getSensorInfo(key);
}
std::string IonosDevice::readSensor(const int, const size_t, const std::string &key) const { return readSensor(key); }

// ============================ settings =======================================

SoapySDR::ArgInfoList IonosDevice::getSettingInfo() const {
    SoapySDR::ArgInfoList l;
    SoapySDR::ArgInfo a;
    a.key = "zero_fill";
    a.name = "Zero-fill lost blocks";
    a.type = SoapySDR::ArgInfo::BOOL;
    a.value = _zeroFill ? "true" : "false";
    a.description = "Insert zeros for missing IQB2 blocks so timing stays continuous";
    l.push_back(a);
    a = SoapySDR::ArgInfo();
    a.key = "report_gaps";
    a.name = "Report gaps as overflow";
    a.type = SoapySDR::ArgInfo::BOOL;
    a.value = _reportGaps ? "true" : "false";
    a.description = "readStream returns SOAPY_SDR_OVERFLOW once per seq gap";
    l.push_back(a);
    a = SoapySDR::ArgInfo();
    a.key = "mode";
    a.name = "Device mode";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "USB";
    a.options = {"USB", "STANDALONE"};
    a.description = "MODE USB stops the on-board demodulators";
    l.push_back(a);
    a = SoapySDR::ArgInfo();
    a.key = "stat_period";
    a.name = "Status period";
    a.type = SoapySDR::ArgInfo::INT;
    a.value = "2";
    a.units = "s";
    a.range = SoapySDR::Range(0, 3600);
    a.description = "MSG1 status period (STAT), 0 = off";
    l.push_back(a);
    a = SoapySDR::ArgInfo();
    a.key = "command";
    a.name = "Raw host command";
    a.type = SoapySDR::ArgInfo::STRING;
    a.description = "Send one host-API line (e.g. o482); read back the last reply. TX/diagnostic letters are refused.";
    l.push_back(a);
    return l;
}

void IonosDevice::writeSetting(const std::string &key, const std::string &value) {
    const bool on = !(value == "0" || value == "false" || value == "off");
    if (key == "zero_fill") { _zeroFill = on; return; }
    if (key == "report_gaps") { _reportGaps = on; return; }
    if (key == "mode") {
        const std::string r = command(std::string("MODE ") + (value == "USB" || value == "usb" ? "USB" : "STANDALONE"));
        _sentModeUsb = (value == "USB" || value == "usb") && r.compare(0, 2, "OK") == 0;
        return;
    }
    if (key == "stat_period") { command("STAT " + value); return; }
    if (key == "command") {
        if (value.empty()) return;
        if (std::strchr(REJECT_LETTERS, value[0]) || value[0] == 'i') {
            // 'i' toggles the stream on the FG23 and would desynchronise the driver's view
            SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyIonos: '%s' is not allowed from the driver", value.c_str());
            return;
        }
        command(value);
        return;
    }
    SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyIonos: unknown setting %s", key.c_str());
}

std::string IonosDevice::readSetting(const std::string &key) const {
    if (key == "zero_fill") return _zeroFill ? "true" : "false";
    if (key == "report_gaps") return _reportGaps ? "true" : "false";
    std::lock_guard<std::mutex> lk(_infoMutex);
    if (key == "mode") {
        auto it = _status.find("mode");
        return it != _status.end() ? it->second : (_sentModeUsb ? "USB" : "STANDALONE");
    }
    if (key == "command") return _lastResponse;
    if (key == "last_error") return _lastError;
    return "";
}

}  // namespace ionos
