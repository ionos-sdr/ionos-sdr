// SPDX-License-Identifier: MIT
//
// IonosDevice.hpp - SoapySDR device for the Ionos SDR (USB CDC I/Q stream)
//   Copyright (c) 2026 Zoltan Doczi HA7DCD

#pragma once

#include "IonosProtocol.hpp"
#include "IonosSerial.hpp"

#include <SoapySDR/Device.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ionos {

// What the device says about itself (hello / CAP?). Defaults = v1 firmware
// without a hello (spec section 6 rule 5).
struct Capabilities {
    bool fromDevice = false;  // false: v1 defaults
    std::map<std::string, std::string> kv;
    double fsIn = 400000.0;
    std::vector<int> decims{8};
    double freqMin = 430e6, freqMax = 440e6;
    int gainShiftMax = 0;
    bool spectrum = false;

    void fromKeyValues(const std::map<std::string, std::string> &m);
};

class IonosDevice : public SoapySDR::Device, private FrameSink {
public:
    explicit IonosDevice(const SoapySDR::Kwargs &args);
    ~IonosDevice() override;

    // identification
    std::string getDriverKey() const override;
    std::string getHardwareKey() const override;
    SoapySDR::Kwargs getHardwareInfo() const override;

    // channels
    size_t getNumChannels(const int direction) const override;
    bool getFullDuplex(const int, const size_t) const override { return false; }

    // stream
    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const override;
    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const override;
    SoapySDR::Stream *setupStream(const int direction, const std::string &format, const std::vector<size_t> &channels,
                                  const SoapySDR::Kwargs &args) override;
    void closeStream(SoapySDR::Stream *stream) override;
    size_t getStreamMTU(SoapySDR::Stream *stream) const override;
    int activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs, const size_t numElems) override;
    int deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs) override;
    int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs,
                   const long timeoutUs) override;

    // antenna
    std::vector<std::string> listAntennas(const int direction, const size_t channel) const override;
    void setAntenna(const int, const size_t, const std::string &) override {}
    std::string getAntenna(const int direction, const size_t channel) const override;

    // gain: the only "gain" is the FG23 digital shift (G<shift>), 6.02 dB per step
    std::vector<std::string> listGains(const int direction, const size_t channel) const override;
    bool hasGainMode(const int, const size_t) const override { return false; }
    void setGain(const int direction, const size_t channel, const std::string &name, const double value) override;
    double getGain(const int direction, const size_t channel, const std::string &name) const override;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const override;

    // frequency
    void setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency,
                      const SoapySDR::Kwargs &args) override;
    double getFrequency(const int direction, const size_t channel, const std::string &name) const override;
    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const override;

    // sample rate / bandwidth
    void setSampleRate(const int direction, const size_t channel, const double rate) override;
    double getSampleRate(const int direction, const size_t channel) const override;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const override;
    void setBandwidth(const int, const size_t, const double) override {}
    double getBandwidth(const int direction, const size_t channel) const override;
    std::vector<double> listBandwidths(const int direction, const size_t channel) const override;

    // sensors
    std::vector<std::string> listSensors() const override;
    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override;
    std::string readSensor(const std::string &key) const override;
    std::vector<std::string> listSensors(const int direction, const size_t channel) const override;
    SoapySDR::ArgInfo getSensorInfo(const int direction, const size_t channel, const std::string &key) const override;
    std::string readSensor(const int direction, const size_t channel, const std::string &key) const override;

    // settings
    SoapySDR::ArgInfoList getSettingInfo() const override;
    void writeSetting(const std::string &key, const std::string &value) override;
    std::string readSetting(const std::string &key) const override;

    // used by find(): open, wait for the hello, close
    static bool probe(const std::string &path, Capabilities &cap, bool &sawIq, std::string &err);

private:
    // FrameSink (reader thread)
    void onIQ(const BlockHeader &h, const uint8_t *payload) override;
    void onSpectrum(const BlockHeader &h, const uint8_t *payload) override;
    void onMessage(const BlockHeader &h, const char *text, size_t len) override;

    void readerLoop();
    bool handshake(double helloWaitS, double capWaitS);
    // Sends one line, waits for the MSG1 type 0 reply; returns "" on timeout.
    std::string command(const std::string &line, double timeoutS = 1.0);
    void pushSamples(const uint8_t *iq, size_t n);
    void pushZeros(size_t n);
    int pickDecim(double rate) const;
    void sendRaw(const std::string &line);

    SoapySDR::Kwargs _args;
    SerialPort _port;
    FrameParser _parser;
    std::thread _reader;
    std::atomic<bool> _running{false};
    std::atomic<bool> _portError{false};

    // capabilities / identity
    mutable std::mutex _infoMutex;
    Capabilities _cap;
    std::map<std::string, std::string> _ver;
    std::map<std::string, std::string> _status;  // last MSG1 type 2
    std::string _lastResponse, _lastError;
    bool _sentModeUsb = false;

    // commands
    std::mutex _cmdMutex;
    std::mutex _respMutex;
    std::condition_variable _respCv;
    std::deque<std::string> _responses;
    std::atomic<bool> _haveCap{false};
    std::atomic<uint64_t> _iqSeen{0};

    // radio state
    std::atomic<double> _freqHz{0.0};
    std::atomic<int> _curDecim{0};       // from the IQB2 header
    std::atomic<int> _reqDecim{0};
    std::atomic<int> _gainShift{0};
    std::atomic<int> _rssi{-999};

    // stream state (seq tracking is reader-thread only)
    bool _haveLastSeq = false;
    uint32_t _lastSeq = 0;
    uint16_t _lastNsamp = 256;
    int _lastHdrDecim = 0;
    std::atomic<bool> _streamSetup{false};
    std::atomic<bool> _active{false};
    std::atomic<bool> _resetStats{false};
    std::string _format = "CS16";
    std::atomic<bool> _zeroFill{true};
    std::atomic<bool> _reportGaps{true};
    uint32_t _maxFillBlocks = 4096;

    // counters (since activateStream)
    std::atomic<uint64_t> _lostBlocks{0}, _gapEvents{0}, _restarts{0}, _hostOverflows{0}, _blocksIn{0}, _spcBlocks{0},
        _msgBlocks{0}, _filledSamples{0};
    std::atomic<uint32_t> _firstSeq{0}, _lastSeqPub{0};
    std::atomic<bool> _overflowPending{false};
    std::atomic<uint64_t> _resyncBase{0}, _truncBase{0};
    std::atomic<uint64_t> _parserResyncs{0}, _parserTrunc{0};  // published by the reader

    // sample ring, SPSC: reader produces, readStream consumes
    std::vector<int16_t> _ring;  // interleaved I/Q
    size_t _ringN = 0;           // complex samples, power of two
    std::atomic<uint64_t> _wpos{0}, _rpos{0};
    std::mutex _dataMutex;
    std::condition_variable _dataCv;

    // log rate limit (reader thread)
    double _logWindowStart = 0.0;
    int _logCount = 0, _logSuppressed = 0;
    bool logAllowed();
};

}  // namespace ionos
