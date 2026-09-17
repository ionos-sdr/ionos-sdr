#pragma once
// FG23 scan-kepes SpyServer-kliens SDR++-hoz.
// A gyari spyserver_client kiterjesztese: az IQ-uzeneteken tul a
// STREAM_TYPE_FFT / MSG_TYPE_UINT8_FFT uzeneteket is fogadja, es a
// spektrumsort egy callbacknek adja tovabb (a modul tolja a waterfallba).
//
// Protokoll-megallapodas az ESP32-oldali szerverrel (spy_fft.h):
//   SETTING_FFT_FREQUENCY      = kozepfrekvencia [Hz]
//   SETTING_FFT_DECIMATION     = SPAN [Hz]  (nem decimacios index! uint32 elég)
//   SETTING_FFT_DISPLAY_PIXELS = binek szama (nbin)
//   SETTING_FFT_DB_OFFSET      = -floor_dBm  (pl. 130 -> a 0-as bin = -130 dBm)
//   SETTING_FFT_DB_RANGE       = tartomany [dB] (pl. 100 -> 255-os bin = -30 dBm)
//   SETTING_FFT_FORMAT         = STREAM_FORMAT_UINT8
//   MSG_TYPE_UINT8_FFT torzse: nbin darab uint8, dB = floor + v*range/255

#include <utils/networking.h>
#include <spyserver_protocol.h>
#include <dsp/stream.h>
#include <dsp/types.h>
#include <functional>
#include <vector>
#include <mutex>
#include <condition_variable>

namespace fg23 {
    // Egy fogadott spektrumsor: dB-ertekek, es a fejlecbol kiolvasott meta.
    struct FFTLine {
        std::vector<float> db;      // nbin darab, dB (abszolut, floor+range szerint)
        uint32_t seq = 0;
        uint16_t flags = 0;         // MessageType felso 16 bit: kozepfrekvencia / 100 kHz
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

        // FFT-sor callback (a halozati olvaso szalon hivodik!)
        void setFFTHandler(FFTHandler h) { fftHandler = h; }

        // A UINT8_FFT dekodolasahoz: ugyanazt kell tudnia, amit a szervernek kuldtunk.
        void setFFTScale(float floorDb, float rangeDb) { fftFloor = floorDb; fftRange = rangeDb; }

        int computeDigitalGain(int serverBits, int deviceGain, int decimationId);

        SpyServerDeviceInfo devInfo;

        // Statisztika a menuhoz
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
