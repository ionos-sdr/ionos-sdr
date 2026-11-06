// SPDX-License-Identifier: MIT
//
// IonosProtocol.cpp - see IonosProtocol.hpp
//   Copyright (c) 2026 Zoltan Doczi HA7DCD

#include "IonosProtocol.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace ionos {

static const size_t NPOS = (size_t)-1;

BlockHeader decodeHeader(const uint8_t *p) {
    BlockHeader h;
    h.magic = rd32(p + 0);
    h.seq = rd32(p + 4);
    h.nsamp = rd16(p + 8);
    h.decim = rd16(p + 10);
    h.fs = rd32(p + 12);
    return h;
}

bool headerPlausible(const uint8_t *p) {
    const uint32_t magic = rd32(p);
    const uint16_t nsamp = rd16(p + 8);
    const uint16_t decim = rd16(p + 10);
    switch (magic) {
    case MAGIC_IQB2:
        // R in {2, 4, 8k}; accept 1..4096 so a future R = 1 path does not break the parser
        return nsamp >= 1 && nsamp <= IQ_MAX_SAMPLES && decim >= 1 && decim <= 4096 &&
               (rd32(p + 12) & 0xFFFFFu) != 0;
    case MAGIC_SPC1:
        return nsamp <= PAYLOAD_BYTES - 8;  // u32 span, i16 floor, u16 range, nbin x u8
    case MAGIC_MSG1:
        return nsamp <= PAYLOAD_BYTES && decim <= 255;
    default:
        return false;
    }
}

FrameParser::FrameParser(size_t capacity) : _buf(std::max(capacity, 4 * BLK_BYTES)) {}

void FrameParser::reset() { _rd = _wr = 0; }

uint8_t *FrameParser::prepare(size_t want) {
    if (_buf.size() - _wr < want) {
        const size_t keep = _wr - _rd;
        if (keep && _rd) std::memmove(_buf.data(), _buf.data() + _rd, keep);
        _rd = 0;
        _wr = keep;
        if (_buf.size() - _wr < want) _buf.resize(_wr + want);
    }
    return _buf.data() + _wr;
}

void FrameParser::commit(size_t n, FrameSink &sink) {
    _wr += n;
    parse(sink);
}

void FrameParser::feed(const uint8_t *data, size_t n, FrameSink &sink) {
    std::memcpy(prepare(n), data, n);
    commit(n, sink);
}

size_t FrameParser::findMagic(size_t from, size_t to) const {
    const uint8_t *b = _buf.data();
    for (size_t i = from; i < to; i++) {
        const uint8_t c = b[i];
        if ((c == 'I' || c == 'S' || c == 'M') && headerPlausible(b + i)) return i;
    }
    return NPOS;
}

void FrameParser::deliver(const uint8_t *p, FrameSink &sink) {
    const BlockHeader h = decodeHeader(p);
    blocks++;
    switch (h.magic) {
    case MAGIC_IQB2: sink.onIQ(h, p + HDR_BYTES); break;
    case MAGIC_SPC1: sink.onSpectrum(h, p + HDR_BYTES); break;
    case MAGIC_MSG1: sink.onMessage(h, (const char *)p + HDR_BYTES, std::min<size_t>(h.nsamp, PAYLOAD_BYTES)); break;
    default: break;
    }
}

void FrameParser::parse(FrameSink &sink) {
    uint8_t *b = _buf.data();
    while (_wr - _rd >= BLK_BYTES) {
        const uint8_t *p = b + _rd;
        if (!headerPlausible(p)) {
            // Lost framing: search byte by byte (any alignment - the 16-byte
            // alignment of the spec only holds relative to the device's
            // stream start, not to where the host happened to start reading).
            resyncs++;
            const size_t q = findMagic(_rd + 1, _wr - HDR_BYTES + 1);
            if (q == NPOS) {
                const size_t nr = _wr - (HDR_BYTES - 1);  // a header may straddle the end
                skippedBytes += nr - _rd;
                _rd = nr;
                break;
            }
            skippedBytes += q - _rd;
            _rd = q;
            continue;
        }
        const size_t avail = _wr - _rd;
        // Fast path: the next block's header sits exactly where it should.
        if (avail >= BLK_BYTES + HDR_BYTES && headerPlausible(p + BLK_BYTES)) {
            deliver(p, sink);
            _rd += BLK_BYTES;
            continue;
        }
        // Slow path (last block in the buffer, or framing damage): a header
        // inside this block's span means the block was cut short (the link
        // occasionally loses USB packets, e.g. 2 x 64 B mid-block). Drop the
        // short block; its seq gap is booked as a lost block downstream.
        const size_t spanEnd = _rd + BLK_BYTES;
        const size_t fullEnd = std::min(spanEnd, _wr - HDR_BYTES + 1);
        const size_t q = findMagic(_rd + 4, fullEnd);
        if (q != NPOS) {
            truncated++;
            skippedBytes += q - _rd;
            _rd = q;
            continue;
        }
        if (avail < BLK_BYTES + HDR_BYTES) {
            // A header could start in the last 15 visible bytes of the span;
            // if a magic is visible there, wait for more data to decide.
            bool maybe = false;
            for (size_t i = fullEnd; i + 4 <= std::min(spanEnd, _wr); i++) {
                const uint32_t m = rd32(b + i);
                if (m == MAGIC_IQB2 || m == MAGIC_SPC1 || m == MAGIC_MSG1) { maybe = true; break; }
            }
            if (maybe) break;
        }
        deliver(p, sink);
        _rd += BLK_BYTES;
    }
    if (_rd == _wr) _rd = _wr = 0;
}

std::map<std::string, std::string> parseKeyValues(const std::string &text) {
    std::map<std::string, std::string> kv;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        const size_t c = line.find(';');
        if (c != std::string::npos) line.erase(c);
        std::istringstream toks(line);
        std::string t;
        while (toks >> t) {
            const size_t e = t.find('=');
            if (e == std::string::npos || e == 0) continue;
            kv[t.substr(0, e)] = t.substr(e + 1);
        }
    }
    return kv;
}

std::vector<int> parseIntList(const std::string &text) {
    std::vector<int> out;
    std::string tok;
    std::istringstream s(text);
    while (std::getline(s, tok, ',')) {
        char *end = nullptr;
        const long v = std::strtol(tok.c_str(), &end, 10);
        if (end != tok.c_str() && v > 0) out.push_back((int)v);
    }
    return out;
}

}  // namespace ionos
