// fg23_scan_source — SDR++ forras-modul a FG23/ESP32 SpyServerhez,
// KET uzemmoddal:
//   IQ   : hagyomanyos keskeny SpyServer-IQ (a demodulacio megy)
//   SCAN : a szerver leptetett szelessavu spektrumsorokat kuld
//          (MSG_TYPE_UINT8_FFT); a modul KOZVETLENUL a gui::waterfall-ba
//          tolja oket. IQ ilyenkor nincs -> nincs hang, csak kep.
//
// Alap: source_modules/spyserver_source (Ryzerth). GPL-3.
// HA7DCD / 2026-08.

#ifndef NOMINMAX
#define NOMINMAX   // windows.h min/max makroi ellen (MSVC)
#endif
#include <fg23_client.h>
#include <imgui.h>
#include <utils/flog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/style.h>
#include <config.h>
#include <gui/smgui.h>
#include <gui/widgets/waterfall.h>
#include <algorithm>
#include <cstring>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "fg23_scan_source",
    /* Description:     */ "FG23 SpyServer source with wideband scan waterfall",
    /* Author:          */ "HA7DCD (based on Ryzerth's spyserver_source)",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};

// ---- IQ formatumok (mint a gyariban) ----
const char* streamFormatStr = "UInt8\0Int16\0Float32\0";
const SpyServerStreamFormat streamFormats[] = { SPYSERVER_STREAM_FORMAT_UINT8, SPYSERVER_STREAM_FORMAT_INT16, SPYSERVER_STREAM_FORMAT_FLOAT };
const int streamFormatsBitCount[] = { 8, 16, 32 };

// ---- SCAN-parameterek ----
// A span-lista: ezt a szerver is ismeri (spy_fft.h), de a protokoll szerint
// Hz-ben kuldjuk, tehat a szerver barmilyen erteket elfogadhat/kerekithet.
const double scanSpans[] = { 2e6, 4e6, 8e6, 16e6, 32e6 };
const char* scanSpansStr = "2 MHz\0" "4 MHz\0" "8 MHz\0" "16 MHz\0" "32 MHz\0";
const int scanSpanCount = 5;

const int scanNbins[] = { 320, 512, 640, 1000 };   // <= 1016 (egy SPECLINE-blokk)
const char* scanNbinsStr = "320\0" "512\0" "640\0" "1000\0";
const int scanNbinCount = 4;

const char* modeStr = "IQ (keskeny, hang)\0" "SCAN (szeles, csak kep)\0";
enum { MODE_IQ = 0, MODE_SCAN = 1 };

ConfigManager config;

class FG23ScanSourceModule : public ModuleManager::Instance {
public:
    FG23ScanSourceModule(std::string name) {
        this->name = name;

        config.acquire();
        std::string host = config.conf["hostname"];
        port = config.conf["port"];
        mode = config.conf["mode"];
        scanSpanId = config.conf["scanSpanId"];
        scanNbinId = config.conf["scanNbinId"];
        fftFloor = config.conf["fftFloorDb"];
        fftRange = config.conf["fftRangeDb"];
        config.release();

        scanSpanId = std::clamp(scanSpanId, 0, scanSpanCount - 1);
        scanNbinId = std::clamp(scanNbinId, 0, scanNbinCount - 1);

        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;

        strcpy(hostname, host.c_str());
        sigpath::sourceManager.registerSource("FG23 Scan", &handler);
    }

    ~FG23ScanSourceModule() {
        stop(this);
        sigpath::sourceManager.unregisterSource("FG23 Scan");
    }

    void postInit() {}
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    bool isEnabled() { return enabled; }

private:
    // ================= WATERFALL-INJEKTALAS =================
    // A halozati szalon hivodik. A waterfall nyers FFT-sora rawFFTSize
    // hosszu (a Display menu "FFT size"-a, alapbol 65536); a nbin-es sort
    // linearisan felhuzzuk erre a hosszra. Az SDR++ sajat FFT-je ilyenkor
    // nem fut (nincs IQ), igy nem ir bele senki mas.
    void onFFTLine(const fg23::FFTLine& L) {
        if (!running || mode != MODE_SCAN) { return; }
        int rawSize = rawFFTSize;
        if (rawSize <= 0) { return; }

        // A szerver a sor kozepfrekvenciajat kuldi (100 kHz egyseg) a flags-ben.
        // Atallas utan az elozo hangolashoz tartozo sorokat ELDOBJUK, kulonben a
        // regi kozeppel mert sor az uj tengely ala kerulne (elcsuszott "torzo").
        if (L.flags != 0) {
            uint16_t want = (uint16_t)(((uint64_t)freq / 100000ull) & 0xFFFFull);
            if (L.flags != want) { staleLines++; return; }
        }
        // Es a binszam is egyezzen a kerttel (nbin-valtas kozben).
        if ((int)L.db.size() != scanNbins[scanNbinId]) { staleLines++; return; }

        float* buf = gui::waterfall.getFFTBuffer();
        if (!buf) { return; }

        const int nbin = (int)L.db.size();
        if (nbin == rawSize) {
            memcpy(buf, L.db.data(), nbin * sizeof(float));
        }
        else if (nbin > rawSize) {
            // Tobb bin, mint pixel: max-decimalas (a csucsok ne vesszenek el)
            for (int i = 0; i < rawSize; i++) {
                int a = (int)((int64_t)i * nbin / rawSize);
                int b = (int)((int64_t)(i + 1) * nbin / rawSize);
                if (b <= a) { b = a + 1; }
                float m = L.db[a];
                for (int j = a + 1; j < b && j < nbin; j++) { m = (std::max)(m, L.db[j]); }
                buf[i] = m;
            }
        }
        else {
            // Kevesebb bin: linearis interpolacio
            const float step = (float)(nbin - 1) / (float)(rawSize - 1);
            for (int i = 0; i < rawSize; i++) {
                float x = i * step;
                int i0 = (int)x;
                int i1 = (std::min)(i0 + 1, nbin - 1);
                float t = x - i0;
                buf[i] = L.db[i0] * (1.0f - t) + L.db[i1] * t;
            }
        }
        gui::waterfall.pushFFT();
    }

    void refreshRawFFTSize() {
        core::configManager.acquire();
        rawFFTSize = core::configManager.conf["fftSize"];
        core::configManager.release();
    }

    // ================= SOURCE HANDLEREK =================
    static void menuSelected(void* ctx) {
        auto* _this = (FG23ScanSourceModule*)ctx;
        core::setInputSampleRate(_this->effectiveSampleRate());
        gui::mainWindow.playButtonLocked = !(_this->client && _this->client->isOpen());
        flog::info("FG23ScanSource '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        auto* _this = (FG23ScanSourceModule*)ctx;
        gui::mainWindow.playButtonLocked = false;
    }

    // SCAN-modban a "mintavetel" = a span, hogy a waterfall tengelye kinyiljon.
    double effectiveSampleRate() {
        return (mode == MODE_SCAN) ? scanSpans[scanSpanId] : sampleRate;
    }

    // SCAN-modban a vizeses dB-tartomanyat a floor/range-hez igazitjuk: az
    // SDR++ alaperteke -70..0 dBFS, a mi abszolut dBm sorunk (-130..-30) az
    // alatt lenne -> csak a legerosebb csucsok latszananak.
    void applyDisplayRange() {
        float lo = fftFloor + 5.0f;
        float hi = fftFloor + fftRange;
        gui::waterfall.setFFTMin(lo);
        gui::waterfall.setFFTMax(hi);
        gui::waterfall.setWaterfallMin(lo);
        gui::waterfall.setWaterfallMax(hi);
    }

    void applyScanSettings() {
        client->setFFTScale(fftFloor, fftRange);
        applyDisplayRange();
        client->setSetting(SPYSERVER_SETTING_FFT_FORMAT, SPYSERVER_STREAM_FORMAT_UINT8);
        client->setSetting(SPYSERVER_SETTING_FFT_FREQUENCY, (uint32_t)freq);
        client->setSetting(SPYSERVER_SETTING_FFT_DECIMATION, (uint32_t)scanSpans[scanSpanId]);   // = SPAN Hz
        client->setSetting(SPYSERVER_SETTING_FFT_DISPLAY_PIXELS, (uint32_t)scanNbins[scanNbinId]);
        client->setSetting(SPYSERVER_SETTING_FFT_DB_OFFSET, (uint32_t)(-fftFloor));
        client->setSetting(SPYSERVER_SETTING_FFT_DB_RANGE, (uint32_t)fftRange);
    }

    static void start(void* ctx) {
        auto* _this = (FG23ScanSourceModule*)ctx;
        if (_this->running) { return; }
        if (!_this->client) {
            _this->tryConnect();
            if (!_this->client) { return; }
        }

        _this->refreshRawFFTSize();

        if (_this->mode == MODE_SCAN) {
            _this->applyScanSettings();
            _this->client->setSetting(SPYSERVER_SETTING_STREAMING_MODE, SPYSERVER_STREAM_MODE_FFT_ONLY);
        }
        else {
            int srvBits = streamFormatsBitCount[_this->iqType];
            int decim = _this->srId + _this->client->devInfo.MinimumIQDecimation;
            _this->client->setSetting(SPYSERVER_SETTING_IQ_FORMAT, streamFormats[_this->iqType]);
            _this->client->setSetting(SPYSERVER_SETTING_IQ_DECIMATION, decim);
            _this->client->setSetting(SPYSERVER_SETTING_IQ_FREQUENCY, (uint32_t)_this->freq);
            _this->client->setSetting(SPYSERVER_SETTING_STREAMING_MODE, SPYSERVER_STREAM_MODE_IQ_ONLY);
            _this->client->setSetting(SPYSERVER_SETTING_GAIN, _this->gain);
            _this->client->setSetting(SPYSERVER_SETTING_IQ_DIGITAL_GAIN, _this->client->computeDigitalGain(srvBits, _this->gain, decim));
        }
        _this->client->startStream();
        _this->running = true;
        flog::info("FG23ScanSource '{0}': Start ({1})", _this->name, _this->mode == MODE_SCAN ? "SCAN" : "IQ");
    }

    static void stop(void* ctx) {
        auto* _this = (FG23ScanSourceModule*)ctx;
        if (!_this->running) { return; }
        _this->running = false;
        _this->client->stopStream();
        flog::info("FG23ScanSource '{0}': Stop!", _this->name);
    }

    static void tune(double freq, void* ctx) {
        auto* _this = (FG23ScanSourceModule*)ctx;
        _this->freq = freq;
        if (_this->running && _this->client) {
            if (_this->mode == MODE_SCAN) {
                _this->client->setSetting(SPYSERVER_SETTING_FFT_FREQUENCY, (uint32_t)freq);
            }
            else {
                _this->client->setSetting(SPYSERVER_SETTING_IQ_FREQUENCY, (uint32_t)freq);
            }
        }
    }

    // Modvaltas futas kozben: leallit, atallit, ujraindit.
    void switchMode(int newMode) {
        bool wasRunning = running;
        if (wasRunning) { stop(this); }
        mode = newMode;
        core::setInputSampleRate(effectiveSampleRate());
        config.acquire();
        config.conf["mode"] = mode;
        config.release(true);
        if (wasRunning) { start(this); }
    }

    // ================= MENU =================
    static void menuHandler(void* ctx) {
        auto* _this = (FG23ScanSourceModule*)ctx;
        bool connected = (_this->client && _this->client->isOpen());
        gui::mainWindow.playButtonLocked = !connected;

        if (connected) { SmGui::BeginDisabled(); }
        if (SmGui::InputText(CONCAT("##_fg23_host_", _this->name), _this->hostname, 1023)) {
            config.acquire(); config.conf["hostname"] = _this->hostname; config.release(true);
        }
        SmGui::SameLine();
        SmGui::FillWidth();
        if (SmGui::InputInt(CONCAT("##_fg23_port_", _this->name), &_this->port, 0, 0)) {
            config.acquire(); config.conf["port"] = _this->port; config.release(true);
        }
        if (connected) { SmGui::EndDisabled(); }

        if (_this->running) { SmGui::BeginDisabled(); }
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (!connected && SmGui::Button("Connect##fg23")) { _this->tryConnect(); }
        else if (connected && SmGui::Button("Disconnect##fg23")) { _this->client->close(); }
        if (_this->running) { SmGui::EndDisabled(); }

        if (!connected) {
            SmGui::Text("Status:"); SmGui::SameLine(); SmGui::Text("Not connected");
            return;
        }

        // ---- Mod ----
        SmGui::LeftLabel("Mode");
        SmGui::FillWidth();
        int m = _this->mode;
        if (SmGui::Combo("##fg23_mode", &m, modeStr)) { _this->switchMode(m); }

        if (_this->mode == MODE_SCAN) {
            SmGui::LeftLabel("Span");
            SmGui::FillWidth();
            if (SmGui::Combo("##fg23_span", &_this->scanSpanId, scanSpansStr)) {
                core::setInputSampleRate(_this->effectiveSampleRate());
                if (_this->running) { _this->applyScanSettings(); }
                config.acquire(); config.conf["scanSpanId"] = _this->scanSpanId; config.release(true);
            }
            SmGui::LeftLabel("Bins");
            SmGui::FillWidth();
            if (SmGui::Combo("##fg23_nbin", &_this->scanNbinId, scanNbinsStr)) {
                if (_this->running) { _this->applyScanSettings(); }
                config.acquire(); config.conf["scanNbinId"] = _this->scanNbinId; config.release(true);
            }
            SmGui::LeftLabel("Floor dBm");
            SmGui::FillWidth();
            if (SmGui::SliderFloat("##fg23_floor", &_this->fftFloor, -150.0f, -60.0f, SmGui::FMT_STR_FLOAT_NO_DECIMAL)) {
                if (_this->running) { _this->applyScanSettings(); }
                config.acquire(); config.conf["fftFloorDb"] = _this->fftFloor; config.release(true);
            }
            SmGui::LeftLabel("Range dB");
            SmGui::FillWidth();
            if (SmGui::SliderFloat("##fg23_range", &_this->fftRange, 20.0f, 150.0f, SmGui::FMT_STR_FLOAT_DB_NO_DECIMAL)) {
                if (_this->running) { _this->applyScanSettings(); }
                config.acquire(); config.conf["fftRangeDb"] = _this->fftRange; config.release(true);
            }
            SmGui::Text("Status:"); SmGui::SameLine();
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "SCAN  lines=%u  (eldobva %u)", _this->client->fftLinesRx, _this->staleLines);
            SmGui::Text("(nincs hang scan-modban)");
        }
        else {
            if (_this->running) { style::beginDisabled(); }
            SmGui::LeftLabel("Samplerate");
            SmGui::FillWidth();
            if (SmGui::Combo("##fg23_sr", &_this->srId, _this->sampleRatesTxt.c_str())) {
                _this->sampleRate = _this->sampleRates[_this->srId];
                core::setInputSampleRate(_this->sampleRate);
                config.acquire(); config.conf["devices"][_this->devRef]["sampleRateId"] = _this->srId; config.release(true);
            }
            if (_this->running) { style::endDisabled(); }

            SmGui::LeftLabel("Sample bit depth");
            SmGui::FillWidth();
            if (SmGui::Combo("##fg23_type", &_this->iqType, streamFormatStr)) {
                int srvBits = streamFormatsBitCount[_this->iqType];
                int decim = _this->srId + _this->client->devInfo.MinimumIQDecimation;
                _this->client->setSetting(SPYSERVER_SETTING_IQ_FORMAT, streamFormats[_this->iqType]);
                _this->client->setSetting(SPYSERVER_SETTING_IQ_DIGITAL_GAIN, _this->client->computeDigitalGain(srvBits, _this->gain, decim));
                config.acquire(); config.conf["devices"][_this->devRef]["sampleBitDepthId"] = _this->iqType; config.release(true);
            }
            if (_this->client->devInfo.MaximumGainIndex) {
                SmGui::FillWidth();
                if (SmGui::SliderInt("##fg23_gain", (int*)&_this->gain, 0, _this->client->devInfo.MaximumGainIndex)) {
                    int srvBits = streamFormatsBitCount[_this->iqType];
                    int decim = _this->srId + _this->client->devInfo.MinimumIQDecimation;
                    _this->client->setSetting(SPYSERVER_SETTING_GAIN, _this->gain);
                    _this->client->setSetting(SPYSERVER_SETTING_IQ_DIGITAL_GAIN, _this->client->computeDigitalGain(srvBits, _this->gain, decim));
                    config.acquire(); config.conf["devices"][_this->devRef]["gainId"] = _this->gain; config.release(true);
                }
            }
            SmGui::Text("Status:"); SmGui::SameLine();
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "IQ  msgs=%u", _this->client->iqMsgsRx);
        }
    }

    // ================= KAPCSOLODAS =================
    std::string bwScaled(double bw) {
        char buf[64];
        if (bw >= 1e6) { sprintf(buf, "%.1lfMHz", bw / 1e6); }
        else if (bw >= 1e3) { sprintf(buf, "%.1lfKHz", bw / 1e3); }
        else { sprintf(buf, "%.1lfHz", bw); }
        return buf;
    }

    void tryConnect() {
        try {
            if (client) { client.reset(); }
            client = fg23::connect(hostname, port, &stream);
            if (!client) { flog::error("fg23: connect failed"); return; }
            client->setFFTHandler([this](const fg23::FFTLine& L) { onFFTLine(L); });
            client->setFFTScale(fftFloor, fftRange);

            if (!client->waitForDevInfo(3000)) {
                flog::error("fg23: no device info");
                return;
            }
            char buf[64];
            sprintf(buf, "FG23 [%08X]", client->devInfo.DeviceSerial);
            devRef = buf;

            config.acquire();
            if (!config.conf["devices"].contains(devRef)) {
                config.conf["devices"][devRef]["sampleRateId"] = 0;
                config.conf["devices"][devRef]["sampleBitDepthId"] = 1;
                config.conf["devices"][devRef]["gainId"] = 0;
            }
            srId = config.conf["devices"][devRef]["sampleRateId"];
            iqType = config.conf["devices"][devRef]["sampleBitDepthId"];
            gain = config.conf["devices"][devRef]["gainId"];
            config.release(true);
            gain = std::clamp<int>(gain, 0, client->devInfo.MaximumGainIndex);

            sampleRates.clear(); sampleRatesTxt.clear();
            for (uint32_t i = client->devInfo.MinimumIQDecimation; i <= client->devInfo.DecimationStageCount; i++) {
                double sr = (double)client->devInfo.MaximumSampleRate / (double)(1 << i);
                sampleRates.push_back(sr);
                sampleRatesTxt += bwScaled(sr); sampleRatesTxt += '\0';
            }
            srId = std::clamp<int>(srId, 0, (int)sampleRates.size() - 1);
            sampleRate = sampleRates[srId];
            core::setInputSampleRate(effectiveSampleRate());
            flog::info("fg23: connected");
        }
        catch (const std::exception& e) {
            flog::error("fg23: connect error {}", e.what());
        }
    }

    // ================= ALLAPOT =================
    std::string name;
    bool enabled = true;
    bool running = false;
    double sampleRate = 1000000;
    double freq = 433.775e6;

    char hostname[1024];
    int port = 5555;
    int iqType = 1;
    int srId = 0;
    std::vector<double> sampleRates;
    std::string sampleRatesTxt;
    uint32_t gain = 0;
    std::string devRef;

    int mode = MODE_IQ;
    int scanSpanId = 2;   // 8 MHz
    int scanNbinId = 0;   // 320
    float fftFloor = -130.0f;
    float fftRange = 100.0f;
    int rawFFTSize = 65536;
    uint32_t staleLines = 0;   // eldobott (regi hangolasu / mas nbin) sorok

    dsp::stream<dsp::complex_t> stream;
    SourceManager::SourceHandler handler;
    fg23::Client client;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    def["hostname"] = "192.168.1.100";
    def["port"] = 5555;
    def["devices"] = json::object();
    def["mode"] = 0;
    def["scanSpanId"] = 2;
    def["scanNbinId"] = 0;
    def["fftFloorDb"] = -130.0f;
    def["fftRangeDb"] = 100.0f;
    config.setPath(core::args["root"].s() + "/fg23_scan_config.json");
    config.load(def);
    config.enableAutoSave();

    config.acquire();
    bool corrected = false;
    for (auto& [k, v] : def.items()) {
        if (!config.conf.contains(k)) { config.conf[k] = v; corrected = true; }
    }
    config.release(corrected);
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new FG23ScanSourceModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete (FG23ScanSourceModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
