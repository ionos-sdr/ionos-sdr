#pragma once
// FG23 scan-capable SpyServer client for SDR++.
// An extension of the stock spyserver_client: in addition to the IQ messages
// it also receives STREAM_TYPE_FFT / MSG_TYPE_UINT8_FFT messages and passes
// each spectrum line to a callback (the module pushes it into the waterfall).
//
// Protocol convention with the ESP32-side server (spy_fft.h):
//   SETTING_FFT_FREQUENCY      = centre frequency [Hz]
//   SETTING_FFT_DECIMATION     = SPAN [Hz]  (not a decimation index; uint32 is sufficient)
//   SETTING_FFT_DISPLAY_PIXELS = number of bins (nbin)
//   SETTING_FFT_DB_OFFSET      = -floor_dBm  (e.g. 130 -> bin value 0 = -130 dBm)
//   SETTING_FFT_DB_RANGE       = range [dB] (e.g. 100 -> bin value 255 = -30 dBm)
//   SETTING_FFT_FORMAT         = STREAM_FORMAT_UINT8
//   MSG_TYPE_UINT8_FFT body: nbin uint8 values, dB = floor + v*range/255

#include <utils/networking.h>
#include <spyserver_protocol.h>
#include <dsp/stream.h>
#include <dsp/types.h>
#include <functional>
#include <vector>
#include <mutex>
#include <condition_variable>

namespace fg23 {
    // One received spectrum line: dB values plus metadata taken from the header.
    struct FFTLine {
        std::vector<float> db;      // nbin values, dB (absolute, per floor+range)
        uint32_t seq = 0;
        uint16_t flags = 0;         // MessageType upper 16 bits: centre frequency / 100 kHz
    };

    typedef std::function<void(const FFTLine&)> FFTHandler;

    class ClientClass {
    public:
        ClientClass(net::Conn conn, dsp::stream<dsp::complex_t>* out);
        ~ClientClass();

        bool waitForDevInfo(int timeoutMS);

        void startStream();
        void stopStream();

        void setSetting(uint32_t setting, uint32_t arg);

        void close();
        bool isOpen();

        // FFT line callback (invoked on the network reader thread!)
        void setFFTHandler(FFTHandler h) { fftHandler = h; }

        // For decoding UINT8_FFT: must match the values sent to the server.
        void setFFTScale(float floorDb, float rangeDb) { fftFloor = floorDb; fftRange = rangeDb; }

        int computeDigitalGain(int serverBits, int deviceGain, int decimationId);

        SpyServerDeviceInfo devInfo;

        // Statistics for the menu
        uint32_t fftLinesRx = 0;
        uint32_t iqMsgsRx = 0;

    private:
        void sendCommand(uint32_t command, void* data, int len);
        void sendHandshake(std::string appName);
        int readSize(int count, uint8_t* buffer);
        static void dataHandler(int count, uint8_t* buf, void* ctx);

        net::Conn client;
        uint8_t* readBuf;
        uint8_t* writeBuf;

        bool deviceInfoAvailable = false;
        std::mutex deviceInfoMtx;
        std::condition_variable deviceInfoCnd;

        SpyServerMessageHeader receivedHeader;
        dsp::stream<dsp::complex_t>* output;

        FFTHandler fftHandler;
        FFTLine fftLine;
        float fftFloor = -130.0f;
        float fftRange = 100.0f;
    };

    typedef std::unique_ptr<ClientClass> Client;

    Client connect(std::string host, uint16_t port, dsp::stream<dsp::complex_t>* out);
}
