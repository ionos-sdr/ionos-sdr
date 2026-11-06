// SPDX-License-Identifier: MIT
//
// parser_test.cpp - FrameParser unit test (no hardware, no SoapySDR)
//   Copyright (c) 2026 Zoltan Doczi HA7DCD
//
// Builds a synthetic device byte stream with junk, dropped blocks, blocks cut
// short by 128 bytes (two lost 64-byte USB packets), SPC1 and MSG1 blocks, and
// feeds it in random chunk sizes. Every intact IQB2 block must come out once,
// in order; every damaged one must be dropped.

#include "IonosProtocol.hpp"

#include <cstdio>
#include <cstring>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace ionos;

static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static std::vector<uint8_t> block(uint32_t magic, uint32_t seq, uint16_t nsamp, uint16_t decim, uint32_t fs,
                                  std::mt19937 &rng) {
    std::vector<uint8_t> b(BLK_BYTES, 0);
    put32(&b[0], magic);
    put32(&b[4], seq);
    put16(&b[8], nsamp);
    put16(&b[10], decim);
    put32(&b[12], fs);
    if (magic == MAGIC_IQB2)
        for (size_t i = HDR_BYTES; i < BLK_BYTES; i++) b[i] = (uint8_t)rng();
    return b;
}

struct Sink : FrameSink {
    std::vector<uint32_t> iq;
    std::vector<std::string> msgs;
    int spc = 0;
    int rssi = 0;
    void onIQ(const BlockHeader &h, const uint8_t *p) override {
        // payload check: first int16 encodes seq low bits (written by the generator)
        uint32_t tag;
        std::memcpy(&tag, p, 4);
        if (tag != (h.seq ^ 0xA5A5A5A5u)) {
            std::printf("payload mismatch at seq %u\n", h.seq);
            iq.push_back(0xFFFFFFFFu);
            return;
        }
        rssi = h.rssiDbm();
        iq.push_back(h.seq);
    }
    void onSpectrum(const BlockHeader &, const uint8_t *) override { spc++; }
    void onMessage(const BlockHeader &, const char *t, size_t n) override { msgs.emplace_back(t, n); }
};

static int runOnce(unsigned seed, bool verbose) {
    std::mt19937 rng(seed);
    std::vector<uint8_t> stream;
    std::vector<uint32_t> expect;
    std::set<uint32_t> damaged;
    // junk prefix: a partial block tail as after opening the port mid-stream
    for (int i = 0; i < 333; i++) stream.push_back((uint8_t)rng());
    const uint32_t N = 3000;
    const int rssi = -97;
    const uint32_t fs = 400000u | ((uint32_t)(rssi & 0xFFF) << 20);
    int msgs = 0, spcs = 0;
    for (uint32_t s = 0; s < N; s++) {
        auto b = block(MAGIC_IQB2, s, 256, 2, fs, rng);
        put32(&b[HDR_BYTES], s ^ 0xA5A5A5A5u);
        const uint32_t r = rng() % 1000;
        if (r < 5) continue;  // dropped block: seq gap
        if (r < 10) {         // two 64-byte USB packets lost inside the block
            const size_t at = 64 + 64 * (rng() % 13);
            b.erase(b.begin() + at, b.begin() + at + 128);
            damaged.insert(s);
        } else {
            expect.push_back(s);
        }
        stream.insert(stream.end(), b.begin(), b.end());
        if (r >= 10 && r < 20) {
            auto m = block(MAGIC_MSG1, 0, 5, MSG_STATUS, 0, rng);
            std::memcpy(&m[HDR_BYTES], "lost=", 5);
            stream.insert(stream.end(), m.begin(), m.end());
            msgs++;
        } else if (r >= 20 && r < 25) {
            auto m = block(MAGIC_SPC1, 0, 512, 1, 433775000u, rng);
            stream.insert(stream.end(), m.begin(), m.end());
            spcs++;
        }
    }
    FrameParser parser(32 * 1024);
    Sink sink;
    size_t off = 0;
    while (off < stream.size()) {
        size_t n = 1 + rng() % 20000;
        if (rng() % 4 == 0) n = 1 + rng() % 40;  // tiny reads too
        n = std::min(n, stream.size() - off);
        parser.feed(&stream[off], n, sink);
        off += n;
    }
    // the very last block stays pending only if a magic could follow; push one more header
    auto tail = block(MAGIC_IQB2, N, 256, 2, fs, rng);
    put32(&tail[HDR_BYTES], N ^ 0xA5A5A5A5u);
    parser.feed(tail.data(), tail.size(), sink);
    expect.push_back(N);

    bool ok = sink.iq == expect && (int)sink.msgs.size() == msgs && sink.spc == spcs &&
              parser.truncated == damaged.size() && sink.rssi == rssi;
    if (verbose || !ok)
        std::printf("seed %u: iq %zu/%zu msg %zu/%d spc %d/%d truncated %llu/%zu resyncs %llu rssi %d -> %s\n", seed,
                    sink.iq.size(), expect.size(), sink.msgs.size(), msgs, sink.spc, spcs,
                    (unsigned long long)parser.truncated, damaged.size(), (unsigned long long)parser.resyncs, sink.rssi,
                    ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}

int main() {
    int fails = 0;
    for (unsigned seed = 1; seed <= 50; seed++) fails += runOnce(seed, seed <= 3);

    // key=value parsing
    auto kv = parseKeyValues("ionos=1\nfreq_min_hz=430000000     ; comment x=1\ndecim_list=2,4,8\nblk_s=781.2 lost=3");
    const auto d = parseIntList(kv["decim_list"]);
    const bool kvOk = kv["ionos"] == "1" && kv["freq_min_hz"] == "430000000" && !kv.count("x") && kv["lost"] == "3" &&
                      d.size() == 3 && d[2] == 8;
    std::printf("key/value parsing: %s\n", kvOk ? "OK" : "FAIL");
    fails += kvOk ? 0 : 1;

    std::printf("%s (%d failures)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails ? 1 : 0;
}
