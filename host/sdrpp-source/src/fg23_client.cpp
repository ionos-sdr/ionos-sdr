#include <fg23_client.h>
#include <volk/volk.h>
#include <cstring>
#include <chrono>
#include <cmath>
#include <utils/flog.h>

using namespace std::chrono_literals;

namespace fg23 {
    ClientClass::ClientClass(net::Conn conn, dsp::stream<dsp::complex_t>* out) {
        readBuf = new uint8_t[SPYSERVER_MAX_MESSAGE_BODY_SIZE];
        writeBuf = new uint8_t[SPYSERVER_MAX_MESSAGE_BODY_SIZE];
        client = std::move(conn);
        output = out;
        output->clearWriteStop();
        sendHandshake("SDR++ fg23scan");
        client->readAsync(sizeof(SpyServerMessageHeader), (uint8_t*)&receivedHeader, dataHandler, this);
    }

    ClientClass::~ClientClass() {
        close();
        delete[] readBuf;
        delete[] writeBuf;
    }

    void ClientClass::startStream() {
        output->clearWriteStop();
        setSetting(SPYSERVER_SETTING_STREAMING_ENABLED, true);
    }

    void ClientClass::stopStream() {
        output->stopWriter();
        setSetting(SPYSERVER_SETTING_STREAMING_ENABLED, false);
    }

    void ClientClass::close() {
        output->stopWriter();
        client->close();
    }

    bool ClientClass::isOpen() {
        return client->isOpen();
    }

    int ClientClass::computeDigitalGain(int serverBits, int deviceGain, int decimationId) {
        // A FG23-szerver DeviceType-ja tetszoleges (RTLSDR-t hirdetunk a
        // kompatibilitasert); a digitalis gain csak a decimaciotol fugg.
        return decimationId * 3.01f;
    }

    bool ClientClass::waitForDevInfo(int timeoutMS) {
        std::unique_lock lck(deviceInfoMtx);
        auto now = std::chrono::system_clock::now();
        deviceInfoCnd.wait_until(lck, now + (timeoutMS * 1ms), [this]() { return deviceInfoAvailable; });
        return deviceInfoAvailable;
    }

    void ClientClass::sendCommand(uint32_t command, void* data, int len) {
        SpyServerCommandHeader* hdr = (SpyServerCommandHeader*)writeBuf;
        hdr->CommandType = command;
        hdr->BodySize = len;
        memcpy(&writeBuf[sizeof(SpyServerCommandHeader)], data, len);
        client->write(sizeof(SpyServerCommandHeader) + len, writeBuf);
    }

    void ClientClass::sendHandshake(std::string appName) {
        int totSize = sizeof(SpyServerClientHandshake) + appName.size();
        uint8_t* buf = new uint8_t[totSize];
        SpyServerClientHandshake* hs = (SpyServerClientHandshake*)buf;
        hs->ProtocolVersion = SPYSERVER_PROTOCOL_VERSION;
        memcpy(&buf[sizeof(SpyServerClientHandshake)], appName.c_str(), appName.size());
        sendCommand(SPYSERVER_CMD_HELLO, buf, totSize);
        delete[] buf;
    }

    void ClientClass::setSetting(uint32_t setting, uint32_t arg) {
        SpyServerSettingTarget target;
        target.Setting = setting;
        target.Value = arg;
        sendCommand(SPYSERVER_CMD_SET_SETTING, &target, sizeof(SpyServerSettingTarget));
    }

    int ClientClass::readSize(int count, uint8_t* buffer) {
        int read = 0;
        int len = 0;
        while (read < count) {
            len = client->read(count - read, &buffer[read]);
            if (len <= 0) { return len; }
            read += len;
        }
        return read;
    }

    void ClientClass::dataHandler(int count, uint8_t* buf, void* ctx) {
        ClientClass* _this = (ClientClass*)ctx;

        if (count < (int)sizeof(SpyServerMessageHeader)) {
            _this->readSize(sizeof(SpyServerMessageHeader) - count, &buf[count]);
        }

        // Vedelem: a szerver ne tudjon a puffernel nagyobbat igerni
        if (_this->receivedHeader.BodySize > SPYSERVER_MAX_MESSAGE_BODY_SIZE) {
            flog::error("fg23: tul nagy uzenettorzs ({0}), bontas", _this->receivedHeader.BodySize);
            _this->client->close();
            return;
        }

        int size = _this->readSize(_this->receivedHeader.BodySize, _this->readBuf);
        if (size <= 0) {
            flog::warn("fg23: kapcsolat bontva");
            return;
        }

        int mtype = _this->receivedHeader.MessageType & 0xFFFF;
        int mflags = (_this->receivedHeader.MessageType & 0xFFFF0000) >> 16;

        if (mtype == SPYSERVER_MSG_TYPE_DEVICE_INFO) {
            {
                std::lock_guard lck(_this->deviceInfoMtx);
                _this->devInfo = *(SpyServerDeviceInfo*)_this->readBuf;
                _this->deviceInfoAvailable = true;
            }
            _this->deviceInfoCnd.notify_all();
        }
        // ---------------- IQ (valtozatlan a gyarihoz kepest) ----------------
        else if (mtype == SPYSERVER_MSG_TYPE_UINT8_IQ) {
            int sampCount = _this->receivedHeader.BodySize / 2;
            float gain = pow(10, (double)mflags / 20.0);
            float scale = 1.0f / (gain * 128.0f);
            for (int i = 0; i < sampCount; i++) {
                _this->output->writeBuf[i].re = ((float)_this->readBuf[2 * i] - 128.0f) * scale;
                _this->output->writeBuf[i].im = ((float)_this->readBuf[2 * i + 1] - 128.0f) * scale;
            }
            _this->output->swap(sampCount);
            _this->iqMsgsRx++;
        }
        else if (mtype == SPYSERVER_MSG_TYPE_INT16_IQ) {
            int sampCount = _this->receivedHeader.BodySize / 4;
            float gain = pow(10, (double)mflags / 20.0);
            volk_16i_s32f_convert_32f((float*)_this->output->writeBuf, (int16_t*)_this->readBuf, 32768.0 * gain, sampCount * 2);
            _this->output->swap(sampCount);
            _this->iqMsgsRx++;
        }
        else if (mtype == SPYSERVER_MSG_TYPE_FLOAT_IQ) {
            int sampCount = _this->receivedHeader.BodySize / sizeof(dsp::complex_t);
            float gain = pow(10, (double)mflags / 20.0);
            volk_32f_s32f_multiply_32f((float*)_this->output->writeBuf, (float*)_this->readBuf, gain, sampCount * 2);
            _this->output->swap(sampCount);
            _this->iqMsgsRx++;
        }
        // ---------------- FFT: EZ AZ UJ RESZ ----------------
        else if (mtype == SPYSERVER_MSG_TYPE_UINT8_FFT) {
            int nbin = _this->receivedHeader.BodySize;
            if (nbin >= SPYSERVER_MIN_DISPLAY_PIXELS && nbin <= SPYSERVER_MAX_DISPLAY_PIXELS && _this->fftHandler) {
                FFTLine& L = _this->fftLine;
                L.db.resize(nbin);
                const float k = _this->fftRange / 255.0f;
                const float f0 = _this->fftFloor;
                const uint8_t* p = _this->readBuf;
                for (int i = 0; i < nbin; i++) {
                    L.db[i] = f0 + (float)p[i] * k;
                }
                L.seq = _this->receivedHeader.SequenceNumber;
                L.flags = (uint16_t)mflags;
                _this->fftLinesRx++;
                _this->fftHandler(L);
            }
        }
        else if (mtype == SPYSERVER_MSG_TYPE_DINT4_FFT) {
            // 4 bites pakolt valtozat: ket bin egy bajtban (also nibble = elso).
            int nbin = _this->receivedHeader.BodySize * 2;
            if (nbin >= SPYSERVER_MIN_DISPLAY_PIXELS && nbin <= SPYSERVER_MAX_DISPLAY_PIXELS && _this->fftHandler) {
                FFTLine& L = _this->fftLine;
                L.db.resize(nbin);
                const float k = _this->fftRange / 15.0f;
                const float f0 = _this->fftFloor;
                const uint8_t* p = _this->readBuf;
                for (int i = 0; i < nbin / 2; i++) {
                    L.db[2 * i]     = f0 + (float)(p[i] & 0x0F) * k;
                    L.db[2 * i + 1] = f0 + (float)(p[i] >> 4) * k;
                }
                L.seq = _this->receivedHeader.SequenceNumber;
                L.flags = (uint16_t)mflags;
                _this->fftLinesRx++;
                _this->fftHandler(L);
            }
        }
        // CLIENT_SYNC, PONG, INT24_IQ, AF: figyelmen kivul hagyva

        _this->client->readAsync(sizeof(SpyServerMessageHeader), (uint8_t*)&_this->receivedHeader, dataHandler, _this);
    }

    Client connect(std::string host, uint16_t port, dsp::stream<dsp::complex_t>* out) {
        net::Conn conn = net::connect(host, port);
        if (!conn) { return NULL; }
        return Client(new ClientClass(std::move(conn), out));
    }
}
