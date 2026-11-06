// SPDX-License-Identifier: MIT
//
// IonosProtocol.hpp - USB host protocol framing (docs/ionos-usb-protocol.md)
//   Copyright (c) 2026 Zoltan Doczi HA7DCD
//
// Pure, I/O-free: the frame parser takes raw bytes from the CDC port and hands
// whole 1040-byte blocks (IQB2 / SPC1 / MSG1) to a sink. Parsing is by offset
// into one linear buffer that the serial reader fills directly, so the hot path
// is one header compare per block and no per-block copies.

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ionos {

constexpr size_t BLK_BYTES = 1040;  // every block type, header included
constexpr size_t HDR_BYTES = 16;
constexpr size_t PAYLOAD_BYTES = BLK_BYTES - HDR_BYTES;  // 1024
constexpr size_t IQ_MAX_SAMPLES = PAYLOAD_BYTES / 4;     // 256 complex int16

constexpr uint32_t MAGIC_IQB2 = 0x32425149u;  // 'I','Q','B','2'
constexpr uint32_t MAGIC_SPC1 = 0x31435053u;  // 'S','P','C','1'
constexpr uint32_t MAGIC_MSG1 = 0x3147534Du;  // 'M','S','G','1'

enum MsgType : uint16_t { MSG_RESPONSE = 0, MSG_HELLO = 1, MSG_STATUS = 2, MSG_ERROR = 3 };

struct BlockHeader {
    uint32_t magic;
    uint32_t seq;
    uint16_t nsamp;  // IQB2: complex samples, SPC1: nbin, MSG1: text length
    uint16_t decim;  // IQB2: total decimation R, SPC1: flags, MSG1: message type
    uint32_t fs;     // IQB2: bits 0..19 fs_in [Hz], 20..31 RSSI [dBm] (signed 12 bit); SPC1: f_center

    uint32_t fsIn() const { return fs & 0xFFFFFu; }
    int rssiDbm() const {
        int v = (int)((fs >> 20) & 0xFFFu);
        return (v & 0x800) ? v - 0x1000 : v;
    }
};

// Little-endian decode (the protocol is LE; works on any host byte order).
inline uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

BlockHeader decodeHeader(const uint8_t *p);

// Magic plus plausibility of nsamp/decim (spec section 6 rule 1).
bool headerPlausible(const uint8_t *p);

class FrameSink {
public:
    virtual ~FrameSink() = default;
    // payload points into the parser's buffer, valid only during the call,
    // byte aligned only (use memcpy).
    virtual void onIQ(const BlockHeader &h, const uint8_t *payload) = 0;
    virtual void onSpectrum(const BlockHeader &h, const uint8_t *payload) = 0;
    virtual void onMessage(const BlockHeader &h, const char *text, size_t len) = 0;
};

class FrameParser {
public:
    explicit FrameParser(size_t capacity = 256 * 1024);

    // Zero-copy fill: get a pointer to at least `want` free bytes, read into
    // it, then commit() the number of bytes actually read. commit() parses
    // and delivers all complete blocks.
    uint8_t *prepare(size_t want);
    void commit(size_t n, FrameSink &sink);

    // Convenience for tests: copies data in and parses.
    void feed(const uint8_t *data, size_t n, FrameSink &sink);

    void reset();

    // Statistics (owned by the parser thread; read them racy/approximate).
    uint64_t blocks = 0;      // well-formed blocks delivered (all types)
    uint64_t resyncs = 0;     // times the parser had to search for a magic
    uint64_t skippedBytes = 0;// bytes thrown away while resynchronising
    uint64_t truncated = 0;   // blocks dropped because the next magic came early (USB packet loss)

private:
    void parse(FrameSink &sink);
    size_t findMagic(size_t from, size_t to) const;  // first plausible header start in [from, to), or npos
    void deliver(const uint8_t *p, FrameSink &sink);

    std::vector<uint8_t> _buf;
    size_t _rd = 0;  // parse offset
    size_t _wr = 0;  // fill offset
};

// "key=value" tokens separated by whitespace/newlines; ';' starts a comment
// until end of line. Later keys overwrite earlier ones.
std::map<std::string, std::string> parseKeyValues(const std::string &text);

// "2,4,8,16" -> {2,4,8,16}; ignores junk tokens.
std::vector<int> parseIntList(const std::string &text);

}  // namespace ionos
