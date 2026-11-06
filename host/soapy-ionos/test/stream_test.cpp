// SPDX-License-Identifier: MIT
//
// stream_test.cpp - end-to-end SoapySDR test against test/ionos_mock.py
//   Copyright (c) 2026 Zoltan Doczi HA7DCD
//
// Streams N seconds, then checks
//   - the achieved sample rate,
//   - the test tone: every non-zero sample must match one continuous complex
//     exponential (so the zero-fill inserted exactly the right sample count),
//   - every zero run is whole blocks at exactly the seqs the mock dropped/cut,
//   - lost_blocks equals what the mock dropped in [first_seq, last_seq].
// The drop rules must match the mock's (--drop-every/--drop-burst/--cut-every).
//
//   ionos_stream_test --args driver=ionos,port=/tmp/ionos0 --rate 200000 --secs 12
//                     --drop-every 500 --drop-burst 2 --cut-every 777

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>

#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

static const double kPi = 3.14159265358979323846;

struct Opts {
    std::string args = "driver=ionos";
    double rate = 200000, secs = 12, tone = 13370, freq = 433775000, amp = 8000;
    long dropEvery = 0, dropBurst = 1, cutEvery = 0;
    std::string format = SOAPY_SDR_CF32;
};

static bool mockLost(const Opts &o, uint64_t s) {
    if (o.dropEvery > 0) {
        const long m = (long)(s % (uint64_t)o.dropEvery), a = o.dropEvery / 2;
        if (m >= a && m < a + o.dropBurst) return true;
    }
    if (o.cutEvery > 0 && (long)(s % (uint64_t)o.cutEvery) == 7) return true;
    return false;
}

int main(int argc, char **argv) {
    Opts o;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string k = argv[i], v = argv[i + 1];
        if (k == "--args") o.args = v;
        else if (k == "--rate") o.rate = std::atof(v.c_str());
        else if (k == "--secs") o.secs = std::atof(v.c_str());
        else if (k == "--tone") o.tone = std::atof(v.c_str());
        else if (k == "--freq") o.freq = std::atof(v.c_str());
        else if (k == "--amp") o.amp = std::atof(v.c_str());
        else if (k == "--drop-every") o.dropEvery = std::atol(v.c_str());
        else if (k == "--drop-burst") o.dropBurst = std::atol(v.c_str());
        else if (k == "--cut-every") o.cutEvery = std::atol(v.c_str());
        else if (k == "--format") o.format = v;
        else { std::fprintf(stderr, "unknown option %s\n", k.c_str()); return 2; }
    }

    SoapySDR::Device *dev = SoapySDR::Device::make(o.args);
    std::printf("hardware: %s\n", dev->getHardwareKey().c_str());
    for (const auto &p : dev->getHardwareInfo()) std::printf("  %s=%s\n", p.first.c_str(), p.second.c_str());
    std::printf("rates:");
    for (double r : dev->listSampleRates(SOAPY_SDR_RX, 0)) std::printf(" %.1f", r);
    std::printf("\n");

    dev->setFrequency(SOAPY_SDR_RX, 0, o.freq);
    dev->setSampleRate(SOAPY_SDR_RX, 0, o.rate);
    // FH / R restart the FG23 stream (seq -> 0) a little later; the drop
    // rules are per seq, so start the window after that.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    const double fs = dev->getSampleRate(SOAPY_SDR_RX, 0);
    std::printf("freq %.0f Hz, rate %.1f sps, format %s\n", dev->getFrequency(SOAPY_SDR_RX, 0), fs, o.format.c_str());

    SoapySDR::Stream *st = dev->setupStream(SOAPY_SDR_RX, o.format);
    const size_t want = (size_t)(o.secs * fs);
    std::vector<std::complex<float>> x(want);
    std::vector<int16_t> cs16(2 * 8192);
    std::vector<int8_t> cs8(2 * 8192);
    size_t got = 0;
    long overflows = 0, timeouts = 0, errors = 0;

    dev->activateStream(st);
    const auto t0 = std::chrono::steady_clock::now();
    while (got < want) {
        const size_t n = std::min<size_t>(8192, want - got);
        void *buf;
        if (o.format == SOAPY_SDR_CF32) buf = &x[got];
        else if (o.format == SOAPY_SDR_CS16) buf = cs16.data();
        else buf = cs8.data();
        void *buffs[] = {buf};
        int flags = 0;
        long long tns = 0;
        const int r = dev->readStream(st, buffs, n, flags, tns, 200000);
        if (r == SOAPY_SDR_OVERFLOW) { overflows++; continue; }
        if (r == SOAPY_SDR_TIMEOUT) { if (++timeouts > 20) break; continue; }
        if (r < 0) { if (++errors > 5) break; continue; }
        if (o.format == SOAPY_SDR_CS16)
            for (int i = 0; i < r; i++) x[got + i] = {cs16[2 * i] / 32768.0f, cs16[2 * i + 1] / 32768.0f};
        else if (o.format == SOAPY_SDR_CS8)
            for (int i = 0; i < r; i++) x[got + i] = {cs8[2 * i] / 128.0f, cs8[2 * i + 1] / 128.0f};
        got += (size_t)r;
    }
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    dev->deactivateStream(st);

    auto sensor = [&](const char *k) { return std::strtoull(dev->readSensor(k).c_str(), nullptr, 10); };
    const uint64_t lost = sensor("lost_blocks"), first = sensor("first_seq"), last = sensor("last_seq");
    std::printf("\nread %zu samples in %.2f s = %.0f sps (overflow returns %ld, timeouts %ld, errors %ld)\n", got, elapsed,
                got / elapsed, overflows, timeouts, errors);
    for (const auto &k : dev->listSensors()) std::printf("  sensor %-17s = %s\n", k.c_str(), dev->readSensor(k).c_str());

    int fails = 0;
    auto check = [&](bool ok, const char *what) {
        std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
        if (!ok) fails++;
    };

    // 1. sample count / rate
    check(got == want, "received the requested number of samples");
    check(std::fabs(got / elapsed - fs) < 0.05 * fs, "sample rate within 5 % of the nominal (wall clock)");

    // 2. zero runs: whole blocks at the mock's drop positions
    const float a = (float)(o.amp / 32768.0);
    const float zthr = 1e-6f + (o.format == SOAPY_SDR_CS8 ? 1.0f / 128 : 0.0f);
    size_t zeroBlocks = 0, badRuns = 0, wrongSeq = 0;
    for (size_t n = 0; n < got;) {
        if (std::abs(x[n]) > zthr) { n++; continue; }
        size_t m = n;
        while (m < got && std::abs(x[m]) <= zthr) m++;
        const bool atEnd = (m == got);
        if (n % 256 != 0 || (!atEnd && (m - n) % 256 != 0)) badRuns++;
        for (size_t b = n / 256; b * 256 < m; b++) {
            zeroBlocks++;
            if (!mockLost(o, first + b)) wrongSeq++;
        }
        n = m;
    }
    uint64_t expectInPrefix = 0;
    for (uint64_t b = 0; b * 256 < got; b++) expectInPrefix += mockLost(o, first + b) ? 1 : 0;
    std::printf("zero-filled blocks in the read data: %zu (mock dropped %llu in that range), bad runs %zu, at wrong seq %zu\n",
                zeroBlocks, (unsigned long long)expectInPrefix, badRuns, wrongSeq);
    check(badRuns == 0 && wrongSeq == 0 && zeroBlocks == expectInPrefix, "zero-fill: whole blocks at exactly the dropped seqs");

    // 3. lost_blocks vs the mock's rule over the whole active window
    uint64_t expectLost = 0;
    for (uint64_t s = first + 1; s < last; s++) expectLost += mockLost(o, s) ? 1 : 0;
    std::printf("lost_blocks %llu, mock dropped %llu between seq %llu and %llu\n", (unsigned long long)lost,
                (unsigned long long)expectLost, (unsigned long long)first, (unsigned long long)last);
    check(lost == expectLost, "lost_blocks == blocks the mock dropped (dropped + cut by 128 B)");

    // 4. tone: one continuous exponential through all non-zero samples
    size_t n0 = 0;
    while (n0 < got && std::abs(x[n0]) <= zthr) n0++;
    double sumDphi = 0;
    size_t nd = 0;
    for (size_t n = n0 + 1; n < got; n++)
        if (std::abs(x[n]) > zthr && std::abs(x[n - 1]) > zthr) {
            sumDphi += std::arg(x[n] * std::conj(x[n - 1]));
            nd++;
        }
    const double fEst = nd ? sumDphi / nd * fs / (2 * kPi) : 0;
    const double w = 2 * kPi * o.tone / fs;
    const double phi0 = std::arg(x[n0]) - w * n0;
    double maxErr = 0, maxAmpErr = 0;
    for (size_t n = n0; n < got; n++) {
        if (std::abs(x[n]) <= zthr) continue;
        const std::complex<double> ref = std::polar(1.0, phi0 + w * (double)n);
        const std::complex<double> v(x[n].real(), x[n].imag());
        maxErr = std::max(maxErr, std::fabs(std::arg(v * std::conj(ref))));
        maxAmpErr = std::max(maxAmpErr, std::fabs(std::abs(v) - a) / a);
    }
    // CS8 = int16 >> 8: amplitude 8000 -> ~31 LSB, truncation -> up to ~0.07 rad
    const double phaseTol = o.format == SOAPY_SDR_CS8 ? 0.1 : 0.002;
    std::printf("tone: estimated %.3f Hz (set %.3f), max phase error vs one continuous tone %.5f rad, max amplitude error %.3f %%\n",
                fEst, o.tone, maxErr, 100 * maxAmpErr);
    check(std::fabs(fEst - o.tone) < 0.5, "tone frequency within 0.5 Hz");
    check(maxErr < phaseTol, "tone phase continuous across every gap (zero-fill length exact)");

    // several gaps between two reads coalesce into one OVERFLOW return
    check(expectLost == 0 ? overflows == 0 : (overflows >= 1 && overflows <= (long)sensor("gap_events")),
          "gaps reported as SOAPY_SDR_OVERFLOW (1 per read call at most)");
    check(sensor("host_overflows") == 0, "no host-side ring overflow");
    check(sensor("stream_restarts") == 0, "no stream restart inside the window");

    dev->closeStream(st);
    SoapySDR::Device::unmake(dev);
    std::printf("\n%s\n", fails ? "STREAM TEST FAILED" : "STREAM TEST PASSED");
    return fails ? 1 : 0;
}
