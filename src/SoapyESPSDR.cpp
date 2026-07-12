#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Version.hpp>

#include <curl/curl.h>
#include <json/json.h>
#include <zlib.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr std::size_t UDP_HEADER_BYTES = 52;
constexpr std::size_t IQ_HEADER_BYTES = 52;
constexpr std::size_t IQ_SAMPLES = 1024;
constexpr std::size_t IQ_FRAME_BYTES = IQ_HEADER_BYTES + IQ_SAMPLES * 4 + 4;
constexpr std::size_t MAX_FRAME_BYTES = 64 * 1024;
constexpr std::size_t MAX_QUEUE_BLOCKS = 512;
constexpr uint32_t UDP_VERSION = 1;

int64_t monotonicNanoseconds()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

uint16_t le16(const uint8_t *p)
{
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}

uint32_t le32(const uint8_t *p)
{
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
           (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

int16_t signExtend10(uint32_t value)
{
    value &= 0x3ffu;
    return static_cast<int16_t>((value & 0x200u) ? int32_t(value) - 1024 : int32_t(value));
}

std::string jsonString(const Json::Value &value)
{
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    return Json::writeString(builder, value);
}

Json::Value parseJson(const std::string &text)
{
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value root;
    std::string errors;
    if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors)) {
        throw std::runtime_error("invalid JSON response: " + errors);
    }
    return root;
}

bool jsonContains(const Json::Value &value, const Json::Value &expected)
{
    if (expected.isObject()) {
        if (!value.isObject()) return false;
        for (const auto &name : expected.getMemberNames()) {
            if (!value.isMember(name) || !jsonContains(value[name], expected[name])) return false;
        }
        return true;
    }
    if (expected.isArray()) {
        if (!value.isArray() || value.size() != expected.size()) return false;
        for (Json::ArrayIndex i = 0; i < expected.size(); ++i) {
            if (!jsonContains(value[i], expected[i])) return false;
        }
        return true;
    }
    if (expected.isNumeric() && value.isNumeric()) return value.asDouble() == expected.asDouble();
    return value == expected;
}

class HttpClient {
public:
    HttpClient(std::string host, unsigned port): _host(std::move(host)), _port(port)
    {
        static const int initialized = []() { return curl_global_init(CURL_GLOBAL_DEFAULT); }();
        if (initialized != CURLE_OK) throw std::runtime_error("curl_global_init failed");
    }

    Json::Value get(const std::string &path) const { return request("GET", path, nullptr); }
    Json::Value post(const std::string &path, const Json::Value &body) const { return request("POST", path, &body); }
    Json::Value put(const std::string &path, const Json::Value &body) const { return request("PUT", path, &body); }

private:
    static size_t writeCallback(char *data, size_t size, size_t count, void *opaque)
    {
        auto *response = static_cast<std::string *>(opaque);
        response->append(data, size * count);
        return size * count;
    }

    Json::Value request(const char *method, const std::string &path, const Json::Value *body) const
    {
        std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> curl(curl_easy_init(), curl_easy_cleanup);
        if (!curl) throw std::runtime_error("curl_easy_init failed");
        const std::string url = "http://" + _host + ":" + std::to_string(_port) + path;
        std::string response;
        std::string encoded;
        curl_slist *rawHeaders = nullptr;
        std::unique_ptr<curl_slist, decltype(&curl_slist_free_all)> headers(nullptr, curl_slist_free_all);
        rawHeaders = curl_slist_append(rawHeaders, "Accept: application/json");
        if (body != nullptr) {
            encoded = jsonString(*body);
            rawHeaders = curl_slist_append(rawHeaders, "Content-Type: application/json");
            curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDS, encoded.c_str());
            curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDSIZE, static_cast<long>(encoded.size()));
        }
        headers.reset(rawHeaders);
        curl_easy_setopt(curl.get(), CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl.get(), CURLOPT_CUSTOMREQUEST, method);
        curl_easy_setopt(curl.get(), CURLOPT_HTTPHEADER, headers.get());
        curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, &HttpClient::writeCallback);
        curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl.get(), CURLOPT_CONNECTTIMEOUT_MS, 1500L);
        curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT_MS, 5000L);
        curl_easy_setopt(curl.get(), CURLOPT_NOSIGNAL, 1L);
        const CURLcode result = curl_easy_perform(curl.get());
        if (result != CURLE_OK) {
            throw std::runtime_error("HTTP request to " + url + " failed: " + curl_easy_strerror(result));
        }
        long status = 0;
        curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &status);
        if (status < 200 || status >= 300) {
            throw std::runtime_error("HTTP " + std::to_string(status) + " from " + path + ": " + response);
        }
        return response.empty() ? Json::Value(Json::objectValue) : parseJson(response);
    }

    std::string _host;
    unsigned _port;
};

struct UdpHeader {
    uint32_t epoch = 0;
    uint32_t datagramSequence = 0;
    uint32_t frameSequence = 0;
    uint32_t sourceChunk = 0;
    uint32_t frameBytes = 0;
    uint32_t frameCrc = 0;
    uint32_t fragmentOffset = 0;
    uint16_t fragmentBytes = 0;
    uint8_t fragmentIndex = 0;
    uint8_t fragmentCount = 0;
    std::array<char, 4> frameMagic{};
    uint32_t firmwareDropped = 0;
};

bool parseHeader(const uint8_t *data, std::size_t bytes, UdpHeader &header)
{
    if (bytes < UDP_HEADER_BYTES || std::memcmp(data, "IQU1", 4) != 0 ||
        le16(data + 4) != UDP_VERSION || le16(data + 6) != UDP_HEADER_BYTES) return false;
    const uint32_t expectedHeaderCrc = le32(data + 48);
    const uint32_t actualHeaderCrc = static_cast<uint32_t>(crc32(0, data, 48));
    if (expectedHeaderCrc != actualHeaderCrc) return false;
    header.epoch = le32(data + 8);
    header.datagramSequence = le32(data + 12);
    header.frameSequence = le32(data + 16);
    header.sourceChunk = le32(data + 20);
    header.frameBytes = le32(data + 24);
    header.frameCrc = le32(data + 28);
    header.fragmentOffset = le32(data + 32);
    header.fragmentBytes = le16(data + 36);
    header.fragmentIndex = data[38];
    header.fragmentCount = data[39];
    std::memcpy(header.frameMagic.data(), data + 40, 4);
    header.firmwareDropped = le32(data + 44);
    return header.frameBytes > 0 && header.frameBytes <= MAX_FRAME_BYTES &&
           header.fragmentCount > 0 && header.fragmentIndex < header.fragmentCount &&
           bytes == UDP_HEADER_BYTES + header.fragmentBytes &&
           header.fragmentOffset + header.fragmentBytes <= header.frameBytes;
}

struct PendingFrame {
    UdpHeader first;
    std::vector<uint8_t> data;
    std::vector<bool> received;
    std::size_t receivedCount = 0;
};

struct SampleBlock {
    std::array<int16_t, IQ_SAMPLES * 2> iq{};
    std::size_t offset = 0;
};

struct StreamState {
    std::string format;
    int socketFd = -1;
    uint16_t port = 0;
    std::atomic<bool> active{false};
    std::atomic<bool> stop{false};
    std::thread worker;
    std::mutex mutex;
    std::condition_variable condition;
    std::deque<SampleBlock> queue;
    std::map<std::pair<uint32_t, uint32_t>, PendingFrame> pending;
    uint32_t epoch = 0;
    bool haveEpoch = false;
    uint32_t expectedSource = 0;
    bool haveExpectedSource = false;
    uint32_t minimumFrameSequence = 0;
    bool haveMinimumFrameSequence = false;
    uint32_t lastFirmwareDropped = 0;
    bool haveFirmwareDropped = false;
    bool overflowPending = false;
    std::atomic<uint64_t> datagrams{0};
    std::atomic<uint64_t> invalidDatagrams{0};
    std::atomic<uint64_t> completedFrames{0};
    std::atomic<uint64_t> lostChunks{0};
    std::atomic<uint64_t> firmwareDrops{0};
    std::atomic<uint64_t> queueDrops{0};
    std::atomic<uint64_t> captureRestarts{0};
    std::atomic<int64_t> suppressContinuityUntilNs{0};
};

class EspDevice final : public SoapySDR::Device {
public:
    explicit EspDevice(const SoapySDR::Kwargs &args):
        _host(valueOr(args, "host", "esp-sdr.local")),
        _httpPort(parseUnsigned(valueOr(args, "http_port", "80"), "http_port", 1, 65535)),
        _requestedUdpPort(parseUnsigned(valueOr(args, "udp_port", "0"), "udp_port", 0, 65535)),
        _rxBufferBytes(parseUnsigned(valueOr(args, "rx_buffer_bytes", "33554432"), "rx_buffer_bytes", 65536, 268435456)),
        _http(_host, _httpPort)
    {
        _config = _http.get("/api/v1/config");
        if (!_config.isObject()) throw std::runtime_error("ESP-SDR returned an invalid configuration");
    }

    ~EspDevice() override
    {
        if (_stream != nullptr) {
            try { deactivateStream(reinterpret_cast<SoapySDR::Stream *>(_stream), 0, 0); } catch (...) {}
            closeStream(reinterpret_cast<SoapySDR::Stream *>(_stream));
        }
    }

    std::string getDriverKey() const override { return "espsdr"; }
    std::string getHardwareKey() const override { return "ESP-SDR"; }
    SoapySDR::Kwargs getHardwareInfo() const override
    {
        return {{"vendor", "Espressif"}, {"hardware", "ESP32-S31 Function-CoreBoard"},
                {"host", _host}, {"transport", "HTTP control / UDP IQ"}};
    }
    std::size_t getNumChannels(const int direction) const override { return direction == SOAPY_SDR_RX ? 1 : 0; }
    bool getFullDuplex(const int, const std::size_t) const override { return false; }
    bool hasGainMode(const int direction, const std::size_t channel) const override { checkRx(direction, channel); return true; }
    void setGainMode(const int direction, const std::size_t channel, const bool automatic) override
    {
        checkRx(direction, channel);
        Json::Value patch;
        patch["gain"]["gain_mode"] = automatic ? 0 : 1;
        applyPatch(patch);
    }
    bool getGainMode(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return configUInt("gain", "gain_mode", 1) == 0;
    }
    std::vector<std::string> listGains(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return {"RX Gain"};
    }
    void setGain(const int direction, const std::size_t channel, const double value) override
    {
        setGain(direction, channel, "RX Gain", value);
    }
    void setGain(const int direction, const std::size_t channel, const std::string &name, const double value) override
    {
        checkRx(direction, channel);
        if (name != "RX Gain") throw std::runtime_error("unknown gain element: " + name);
        Json::Value patch;
        patch["gain"]["gain_mode"] = 1;
        patch["gain"]["rx_gain"] = static_cast<unsigned>(std::clamp(std::llround(value), 0ll, 127ll));
        applyPatch(patch);
    }
    double getGain(const int direction, const std::size_t channel) const override
    {
        return getGain(direction, channel, "RX Gain");
    }
    double getGain(const int direction, const std::size_t channel, const std::string &name) const override
    {
        checkRx(direction, channel);
        if (name != "RX Gain") throw std::runtime_error("unknown gain element: " + name);
        return configUInt("gain", "rx_gain", 32);
    }
    SoapySDR::Range getGainRange(const int direction, const std::size_t channel) const override
    {
        return getGainRange(direction, channel, "RX Gain");
    }
    SoapySDR::Range getGainRange(const int direction, const std::size_t channel, const std::string &name) const override
    {
        checkRx(direction, channel);
        if (name != "RX Gain") throw std::runtime_error("unknown gain element: " + name);
        return {0, 127, 1};
    }

    void setFrequency(const int direction, const std::size_t channel, const double frequency,
                      const SoapySDR::Kwargs &) override
    {
        checkRx(direction, channel);
        if (frequency < 2.3e9 || frequency > 2.8e9) throw std::runtime_error("frequency must be 2300-2800 MHz");
        Json::Value patch;
        patch["radio"]["rf_freq_hz"] = Json::UInt64(std::llround(frequency / 1000.0) * 1000);
        applyPatch(patch);
    }
    double getFrequency(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return configUInt64("radio", "rf_freq_hz", 2412000000u);
    }
    std::vector<std::string> listFrequencies(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return {"RF"};
    }
    SoapySDR::RangeList getFrequencyRange(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return {{2.3e9, 2.8e9, 1000}};
    }

    void setSampleRate(const int direction, const std::size_t channel, const double rate) override
    {
        checkRx(direction, channel);
        const unsigned decimation = rateToDecimation(rate);
        Json::Value patch;
        patch["iq_engine"]["adc_decimation"] = decimation;
        applyPatch(patch);
    }
    double getSampleRate(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return 80e6 / configUInt("iq_engine", "adc_decimation", 10);
    }
    std::vector<double> listSampleRates(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return {8e6, 16e6, 20e6};
    }
    SoapySDR::RangeList getSampleRateRange(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return {{8e6, 8e6}, {16e6, 16e6}, {20e6, 20e6}};
    }

    void setBandwidth(const int direction, const std::size_t channel, const double bandwidth) override
    {
        checkRx(direction, channel);
        unsigned mhz = 0;
        if (bandwidth != 0) {
            mhz = static_cast<unsigned>(std::llround(bandwidth / 1e6));
            if (mhz < 13 || mhz > 54) throw std::runtime_error("bandwidth must be 0/open or 13-54 MHz");
        }
        Json::Value patch;
        patch["bandwidth"]["bw_mhz"] = 20;
        patch["bandwidth"]["second_chan"] = 0;
        patch["rx_filter"]["filter_bw_mhz"] = mhz;
        patch["rx_filter"]["rx_filter_override"] = 0;
        applyPatch(patch);
    }
    double getBandwidth(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return configUInt("rx_filter", "filter_bw_mhz", 0) * 1e6;
    }
    std::vector<double> listBandwidths(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        std::vector<double> values{0};
        for (unsigned mhz = 13; mhz <= 54; ++mhz) values.push_back(mhz * 1e6);
        return values;
    }
    SoapySDR::RangeList getBandwidthRange(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return {{13e6, 54e6, 1e6}};
    }

    std::vector<std::string> getStreamFormats(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return {SOAPY_SDR_CS16, SOAPY_SDR_CF32};
    }
    std::string getNativeStreamFormat(const int direction, const std::size_t channel, double &fullScale) const override
    {
        checkRx(direction, channel);
        fullScale = 512.0;
        return SOAPY_SDR_CS16;
    }
    SoapySDR::Stream *setupStream(const int direction, const std::string &format,
                                  const std::vector<std::size_t> &channels,
                                  const SoapySDR::Kwargs &) override
    {
        checkRx(direction, channels.empty() ? 0 : channels.front());
        if (channels.size() > 1) throw std::runtime_error("SoapyESPSDR has one RX channel");
        if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32) throw std::runtime_error("supported formats are CS16 and CF32");
        if (_stream != nullptr) throw std::runtime_error("only one RX stream is supported");
        auto state = std::make_unique<StreamState>();
        state->format = format;
        state->socketFd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (state->socketFd < 0) throw std::runtime_error("failed to create UDP socket");
        int reuse = 1;
        setsockopt(state->socketFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        int requestedBuffer = static_cast<int>(_rxBufferBytes);
        setsockopt(state->socketFd, SOL_SOCKET, SO_RCVBUF, &requestedBuffer, sizeof(requestedBuffer));
        timeval timeout{0, 50000};
        setsockopt(state->socketFd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(static_cast<uint16_t>(_requestedUdpPort));
        if (::bind(state->socketFd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
            ::close(state->socketFd);
            throw std::runtime_error("failed to bind UDP receiver port");
        }
        socklen_t length = sizeof(address);
        if (getsockname(state->socketFd, reinterpret_cast<sockaddr *>(&address), &length) != 0) {
            ::close(state->socketFd);
            throw std::runtime_error("failed to query UDP receiver port");
        }
        state->port = ntohs(address.sin_port);
        _stream = state.release();
        return reinterpret_cast<SoapySDR::Stream *>(_stream);
    }
    void closeStream(SoapySDR::Stream *stream) override
    {
        auto *state = checkedStream(stream);
        if (state->active) deactivateStream(stream, 0, 0);
        if (state->socketFd >= 0) ::close(state->socketFd);
        delete state;
        _stream = nullptr;
    }
    std::size_t getStreamMTU(SoapySDR::Stream *stream) const override
    {
        checkedStream(stream);
        return IQ_SAMPLES;
    }
    int activateStream(SoapySDR::Stream *stream, const int flags, const long long, const std::size_t numElems) override
    {
        auto *state = checkedStream(stream);
        if (flags != 0 || numElems != 0) return SOAPY_SDR_NOT_SUPPORTED;
        if (state->active) return 0;
        Json::Value patch;
        patch["stream"]["output_mode"] = 0;
        patch["stream"]["stream_wifi_packets"] = 0;
        patch["trigger"]["trigger_mode"] = 0;
        Json::Value trigger(Json::arrayValue);
        trigger.append(1); trigger.append(0); trigger.append(1);
        for (unsigned i = 3; i < 16; ++i) trigger.append(0);
        patch["trigger"]["trigger_config"] = trigger;
        applyPatch(patch);
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->queue.clear();
            state->pending.clear();
            state->overflowPending = false;
            state->haveEpoch = false;
            state->haveExpectedSource = false;
            state->haveFirmwareDropped = false;
            state->haveMinimumFrameSequence = false;
        }
        state->stop = false;
        state->active = true;
        state->worker = std::thread(&EspDevice::receiveLoop, state);
        Json::Value body;
        body["port"] = state->port;
        try {
            _http.post("/api/v1/stream/start", body);
        } catch (...) {
            state->stop = true;
            if (state->worker.joinable()) state->worker.join();
            state->active = false;
            throw;
        }
        return 0;
    }
    int deactivateStream(SoapySDR::Stream *stream, const int flags, const long long) override
    {
        auto *state = checkedStream(stream);
        if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
        if (!state->active) return 0;
        try { _http.post("/api/v1/stream/stop", Json::Value(Json::objectValue)); }
        catch (const std::exception &error) { SoapySDR::logf(SOAPY_SDR_WARNING, "stream stop failed: %s", error.what()); }
        state->stop = true;
        if (state->worker.joinable()) state->worker.join();
        state->active = false;
        state->condition.notify_all();
        return 0;
    }
    int readStream(SoapySDR::Stream *stream, void *const *buffers, const std::size_t numElems,
                   int &flags, long long &timeNs, const long timeoutUs) override
    {
        auto *state = checkedStream(stream);
        flags = 0;
        timeNs = 0;
        if (!state->active) {
            if (timeoutUs > 0) std::this_thread::sleep_for(std::chrono::microseconds(timeoutUs));
            return SOAPY_SDR_TIMEOUT;
        }
        std::unique_lock<std::mutex> lock(state->mutex);
        const auto ready = [&]() { return !state->queue.empty() || state->overflowPending || !state->active; };
        if (!state->condition.wait_for(lock, std::chrono::microseconds(std::max<long>(timeoutUs, 0)), ready)) return SOAPY_SDR_TIMEOUT;
        if (state->overflowPending) {
            state->overflowPending = false;
            return SOAPY_SDR_OVERFLOW;
        }
        if (state->queue.empty()) return SOAPY_SDR_TIMEOUT;
        std::size_t produced = 0;
        while (produced < numElems && !state->queue.empty()) {
            SampleBlock &block = state->queue.front();
            const std::size_t count = std::min(numElems - produced, IQ_SAMPLES - block.offset);
            if (state->format == SOAPY_SDR_CS16) {
                auto *output = static_cast<int16_t *>(buffers[0]);
                std::memcpy(output + produced * 2, block.iq.data() + block.offset * 2, count * 2 * sizeof(int16_t));
            } else {
                auto *output = static_cast<float *>(buffers[0]);
                for (std::size_t i = 0; i < count * 2; ++i) output[produced * 2 + i] = block.iq[block.offset * 2 + i] / 512.0f;
            }
            produced += count;
            block.offset += count;
            if (block.offset == IQ_SAMPLES) state->queue.pop_front();
        }
        return static_cast<int>(produced);
    }

    std::vector<std::string> listSensors() const override
    {
        return {"datagrams", "invalid_datagrams", "completed_frames", "lost_chunks",
                "firmware_drops", "queue_drops", "capture_restarts"};
    }
    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override
    {
        const auto sensors = listSensors();
        if (std::find(sensors.begin(), sensors.end(), key) == sensors.end()) throw std::runtime_error("unknown sensor: " + key);
        SoapySDR::ArgInfo info;
        info.key = key; info.name = key; info.value = "0"; info.type = SoapySDR::ArgInfo::INT;
        return info;
    }
    std::string readSensor(const std::string &key) const override
    {
        const StreamState *state = _stream;
        if (state == nullptr) return "0";
        if (key == "datagrams") return std::to_string(state->datagrams.load());
        if (key == "invalid_datagrams") return std::to_string(state->invalidDatagrams.load());
        if (key == "completed_frames") return std::to_string(state->completedFrames.load());
        if (key == "lost_chunks") return std::to_string(state->lostChunks.load());
        if (key == "firmware_drops") return std::to_string(state->firmwareDrops.load());
        if (key == "queue_drops") return std::to_string(state->queueDrops.load());
        if (key == "capture_restarts") return std::to_string(state->captureRestarts.load());
        throw std::runtime_error("unknown sensor: " + key);
    }

private:
    static std::string valueOr(const SoapySDR::Kwargs &args, const std::string &key, const std::string &fallback)
    {
        const auto it = args.find(key);
        return it == args.end() ? fallback : it->second;
    }
    static unsigned parseUnsigned(const std::string &text, const char *name, unsigned minimum, unsigned maximum)
    {
        std::size_t consumed = 0;
        const unsigned long value = std::stoul(text, &consumed, 0);
        if (consumed != text.size() || value < minimum || value > maximum) throw std::runtime_error(std::string(name) + " is out of range");
        return static_cast<unsigned>(value);
    }
    static void checkRx(const int direction, const std::size_t channel)
    {
        if (direction != SOAPY_SDR_RX || channel != 0) throw std::runtime_error("SoapyESPSDR supports RX channel 0 only");
    }
    static unsigned rateToDecimation(double rate)
    {
        if (std::abs(rate - 8e6) < 1) return 10;
        if (std::abs(rate - 16e6) < 1) return 5;
        if (std::abs(rate - 20e6) < 1) return 4;
        throw std::runtime_error("supported sample rates are 8, 16, and 20 MSa/s");
    }
    StreamState *checkedStream(SoapySDR::Stream *stream) const
    {
        auto *state = reinterpret_cast<StreamState *>(stream);
        if (state == nullptr || state != _stream) throw std::runtime_error("invalid SoapyESPSDR stream");
        return state;
    }
    uint64_t configUInt64(const char *group, const char *key, uint64_t fallback) const
    {
        std::lock_guard<std::mutex> lock(_configMutex);
        const Json::Value &value = _config[group][key];
        return value.isNumeric() ? value.asUInt64() : fallback;
    }
    unsigned configUInt(const char *group, const char *key, unsigned fallback) const
    {
        return static_cast<unsigned>(configUInt64(group, key, fallback));
    }
    void applyPatch(const Json::Value &patch)
    {
        std::lock_guard<std::mutex> controlLock(_controlMutex);
        StreamState *stream = _stream;
        const bool active = stream != nullptr && stream->active.load();
        if (active) {
            stream->suppressContinuityUntilNs = monotonicNanoseconds() + 15'000'000'000ll;
        }
        try {
            _http.put("/api/v1/config", patch);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (std::chrono::steady_clock::now() < deadline) {
                const Json::Value status = _http.get("/api/v1/status");
                if (!status.get("config_applying", false).asBool()) {
                    Json::Value applied = _http.get("/api/v1/config");
                    if (jsonContains(applied, patch)) {
                        std::lock_guard<std::mutex> lock(_configMutex);
                        _config = std::move(applied);
                        if (active) {
                            stream->suppressContinuityUntilNs =
                                monotonicNanoseconds() + 500'000'000ll;
                        }
                        return;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            throw std::runtime_error("timed out waiting for ESP-SDR configuration");
        } catch (...) {
            if (active) stream->suppressContinuityUntilNs = 0;
            throw;
        }
    }
    static bool validFrame(const PendingFrame &pending)
    {
        const auto &frame = pending.data;
        if (frame.size() != pending.first.frameBytes || std::memcmp(frame.data(), pending.first.frameMagic.data(), 4) != 0) return false;
        if (pending.first.frameCrc == 0) return true;
        if (frame.size() < 4 || le32(frame.data() + frame.size() - 4) != pending.first.frameCrc) return false;
        return static_cast<uint32_t>(crc32(0, frame.data(), frame.size() - 4)) == pending.first.frameCrc;
    }
    static void finishFrame(StreamState *state, PendingFrame &&pending)
    {
        if (!validFrame(pending) || pending.data.size() != IQ_FRAME_BYTES || std::memcmp(pending.data.data(), "IQC1", 4) != 0) {
            state->invalidDatagrams++;
            return;
        }
        const uint32_t source = le32(pending.data.data() + 8);
        SampleBlock block;
        const uint8_t *sampleData = pending.data.data() + IQ_HEADER_BYTES;
        for (std::size_t i = 0; i < IQ_SAMPLES; ++i) {
            const uint32_t word = le32(sampleData + i * 4);
            // The dump word stores Q in bits 9:0 and I in bits 19:10.
            // Soapy complex formats are interleaved I,Q.
            block.iq[i * 2] = signExtend10(word >> 10);
            block.iq[i * 2 + 1] = signExtend10(word);
        }
        std::lock_guard<std::mutex> lock(state->mutex);
        const bool suppressContinuity = monotonicNanoseconds() < state->suppressContinuityUntilNs.load();
        if (state->haveExpectedSource) {
            if (source < state->expectedSource) {
                // Applying RF configuration restarts capture at a low source
                // index without changing the UDP epoch. Treat only a small
                // backward step as reordering; otherwise begin a new capture
                // generation instead of rejecting the restarted stream forever.
                if (state->expectedSource - source <= 8) return;
                state->queue.clear();
                state->captureRestarts++;
                if (!suppressContinuity) state->overflowPending = true;
            }
            if (source > state->expectedSource) {
                if (!suppressContinuity) {
                    state->lostChunks += source - state->expectedSource;
                    state->overflowPending = true;
                }
            }
        }
        state->expectedSource = source + 1;
        state->haveExpectedSource = true;
        if (state->queue.size() >= MAX_QUEUE_BLOCKS) {
            state->queue.pop_front();
            state->queueDrops++;
            state->overflowPending = true;
        }
        state->queue.emplace_back(std::move(block));
        state->completedFrames++;
        state->condition.notify_one();
    }
    static void handleDatagram(StreamState *state, const uint8_t *data, std::size_t bytes)
    {
        state->datagrams++;
        UdpHeader header;
        if (!parseHeader(data, bytes, header)) { state->invalidDatagrams++; return; }
        if (std::memcmp(header.frameMagic.data(), "IQC1", 4) != 0) return;
        if (!state->haveEpoch || state->epoch != header.epoch) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->epoch = header.epoch;
            state->haveEpoch = true;
            state->pending.clear();
            state->queue.clear();
            state->haveExpectedSource = false;
            state->haveFirmwareDropped = false;
            state->haveMinimumFrameSequence = false;
        }
        bool captureRestart = false;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (state->haveExpectedSource && header.sourceChunk < state->expectedSource &&
                state->expectedSource - header.sourceChunk > 8) {
                state->queue.clear();
                state->haveExpectedSource = false;
                state->captureRestarts++;
                if (monotonicNanoseconds() >= state->suppressContinuityUntilNs.load()) {
                    state->overflowPending = true;
                }
                state->minimumFrameSequence = header.frameSequence;
                state->haveMinimumFrameSequence = true;
                captureRestart = true;
            }
        }
        if (captureRestart) state->pending.clear();
        if (state->haveMinimumFrameSequence &&
            static_cast<int32_t>(header.frameSequence - state->minimumFrameSequence) < 0) return;
        if (state->haveFirmwareDropped && header.firmwareDropped > state->lastFirmwareDropped) {
            state->firmwareDrops += header.firmwareDropped - state->lastFirmwareDropped;
        }
        state->lastFirmwareDropped = header.firmwareDropped;
        state->haveFirmwareDropped = true;
        const auto key = std::make_pair(header.epoch, header.frameSequence);
        auto it = state->pending.find(key);
        if (it == state->pending.end()) {
            PendingFrame pending;
            pending.first = header;
            pending.data.resize(header.frameBytes);
            pending.received.resize(header.fragmentCount, false);
            it = state->pending.emplace(key, std::move(pending)).first;
        }
        PendingFrame &pending = it->second;
        if (pending.data.size() != header.frameBytes || pending.received.size() != header.fragmentCount || pending.received[header.fragmentIndex]) return;
        std::memcpy(pending.data.data() + header.fragmentOffset, data + UDP_HEADER_BYTES, header.fragmentBytes);
        pending.received[header.fragmentIndex] = true;
        pending.receivedCount++;
        if (pending.receivedCount == pending.received.size()) {
            PendingFrame complete = std::move(pending);
            state->pending.erase(it);
            finishFrame(state, std::move(complete));
        }
        while (state->pending.size() > 128) {
            state->pending.erase(state->pending.begin());
            std::lock_guard<std::mutex> lock(state->mutex);
            state->overflowPending = true;
        }
    }
    static void receiveLoop(StreamState *state)
    {
        std::array<uint8_t, 2048> buffer{};
        while (!state->stop) {
            const ssize_t received = recv(state->socketFd, buffer.data(), buffer.size(), 0);
            if (received > 0) handleDatagram(state, buffer.data(), static_cast<std::size_t>(received));
        }
    }

    std::string _host;
    unsigned _httpPort;
    unsigned _requestedUdpPort;
    unsigned _rxBufferBytes;
    HttpClient _http;
    std::mutex _controlMutex;
    mutable std::mutex _configMutex;
    Json::Value _config;
    StreamState *_stream = nullptr;
};

SoapySDR::KwargsList findEspSdr(const SoapySDR::Kwargs &args)
{
    const auto driver = args.find("driver");
    if (driver != args.end() && driver->second != "espsdr") return {};
    SoapySDR::Kwargs result = args;
    result["driver"] = "espsdr";
    if (result.count("host") == 0) result["host"] = "esp-sdr.local";
    const unsigned port = result.count("http_port") ? static_cast<unsigned>(std::stoul(result["http_port"])) : 80;
    try {
        HttpClient http(result["host"], port);
        const Json::Value status = http.get("/api/v1/status");
        result["label"] = "ESP-SDR (" + result["host"] + ")";
        if (status.isMember("ipv4")) result["ipv4"] = status["ipv4"].asString();
        return {result};
    } catch (const std::exception &error) {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyESPSDR discovery at %s failed: %s", result["host"].c_str(), error.what());
        return {};
    }
}

SoapySDR::Device *makeEspSdr(const SoapySDR::Kwargs &args) { return new EspDevice(args); }
static SoapySDR::Registry registerEspSdr("espsdr", &findEspSdr, &makeEspSdr, SOAPY_SDR_ABI_VERSION);

} // namespace
