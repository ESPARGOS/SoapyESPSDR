#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Version.hpp>

#include <curl/curl.h>
#include <json/json.h>
#include <libusb-1.0/libusb.h>
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
constexpr unsigned ADC_CLOCK_HZ = 80'000'000;
constexpr unsigned ADC_DECIMATION_MAX = 10;
constexpr unsigned STREAM_SELECTION_MAX = 1'000'000;

// USB transport: vendor interface on the ESP32-S31 native high-speed port.
constexpr uint16_t USB_VID = 0x303A;
constexpr uint16_t USB_PID = 0x4531;
constexpr const char *USB_PRODUCT = "ESP-SDR";
constexpr uint8_t USB_EP_CTRL_OUT = 0x01;
constexpr uint8_t USB_EP_CTRL_IN = 0x81;
constexpr uint8_t USB_EP_STREAM_IN = 0x82;
constexpr std::size_t USB_CTRL_HEADER_BYTES = 16;
constexpr std::size_t USB_CTRL_MAX_PAYLOAD = 2048;
constexpr unsigned USB_CTRL_TIMEOUT_MS = 3000;
constexpr int USB_STREAM_TRANSFERS = 16;
constexpr std::size_t USB_STREAM_TRANSFER_BYTES = 16 * 1024;

enum UsbControlOpcode : uint32_t {
    USB_OP_GET_STATUS = 1,
    USB_OP_GET_CONFIG = 2,
    USB_OP_PUT_CONFIG = 3,
    USB_OP_STREAM_START = 4,
    USB_OP_STREAM_STOP = 5,
};

Json::Value intervalTrigger(unsigned total, unsigned streamed)
{
    Json::Value trigger(Json::arrayValue);
    trigger.append(total);
    trigger.append(0);
    trigger.append(streamed);
    for (unsigned i = 3; i < 16; ++i) trigger.append(0);
    return trigger;
}

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

// Abstract control plane: HTTP JSON API or the equivalent USB channel.
class Control {
public:
    virtual ~Control() = default;
    virtual Json::Value get(const std::string &path) = 0;
    virtual Json::Value put(const std::string &path, const Json::Value &body) = 0;
    virtual Json::Value post(const std::string &path, const Json::Value &body) = 0;
};

class HttpControl final : public Control {
public:
    HttpControl(std::string host, unsigned port): _http(std::move(host), port) {}
    Json::Value get(const std::string &path) override { return _http.get(path); }
    Json::Value put(const std::string &path, const Json::Value &body) override { return _http.put(path, body); }
    Json::Value post(const std::string &path, const Json::Value &body) override { return _http.post(path, body); }

private:
    HttpClient _http;
};

struct UsbContext {
    libusb_context *context = nullptr;
    libusb_device_handle *handle = nullptr;
    std::string serial;

    ~UsbContext()
    {
        if (handle != nullptr) {
            libusb_release_interface(handle, 0);
            libusb_close(handle);
        }
        if (context != nullptr) libusb_exit(context);
    }
};

std::string usbStringDescriptor(libusb_device_handle *handle, uint8_t index)
{
    if (index == 0) return {};
    unsigned char text[128] = {0};
    const int length = libusb_get_string_descriptor_ascii(handle, index, text, sizeof(text) - 1);
    return length > 0 ? std::string(reinterpret_cast<char *>(text), length) : std::string();
}

// Open the ESP-SDR vendor device, optionally matching a specific serial.
std::shared_ptr<UsbContext> usbOpen(const std::string &serial)
{
    auto usb = std::make_shared<UsbContext>();
    if (libusb_init(&usb->context) != 0) throw std::runtime_error("libusb_init failed");
    libusb_device **list = nullptr;
    const ssize_t count = libusb_get_device_list(usb->context, &list);
    std::string firstError = "no ESP-SDR USB device found";
    for (ssize_t i = 0; i < count && usb->handle == nullptr; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != 0) continue;
        if (desc.idVendor != USB_VID || desc.idProduct != USB_PID) continue;
        libusb_device_handle *handle = nullptr;
        const int status = libusb_open(list[i], &handle);
        if (status != 0) {
            firstError = std::string("cannot open ESP-SDR USB device: ") + libusb_error_name(status);
            continue;
        }
        const std::string product = usbStringDescriptor(handle, desc.iProduct);
        const std::string deviceSerial = usbStringDescriptor(handle, desc.iSerialNumber);
        if (product != USB_PRODUCT || (!serial.empty() && deviceSerial != serial)) {
            libusb_close(handle);
            continue;
        }
        libusb_set_auto_detach_kernel_driver(handle, 1);
        const int claim = libusb_claim_interface(handle, 0);
        if (claim != 0) {
            libusb_close(handle);
            firstError = std::string("cannot claim ESP-SDR USB interface: ") + libusb_error_name(claim);
            continue;
        }
        usb->handle = handle;
        usb->serial = deviceSerial;
    }
    if (list != nullptr) libusb_free_device_list(list, 1);
    if (usb->handle == nullptr) throw std::runtime_error(firstError);
    return usb;
}

class UsbControl final : public Control {
public:
    explicit UsbControl(std::shared_ptr<UsbContext> usb): _usb(std::move(usb)) {}

    Json::Value get(const std::string &path) override
    {
        if (path == "/api/v1/status") return request(USB_OP_GET_STATUS, nullptr);
        if (path == "/api/v1/config") return request(USB_OP_GET_CONFIG, nullptr);
        throw std::runtime_error("unsupported USB control path: " + path);
    }
    Json::Value put(const std::string &path, const Json::Value &body) override
    {
        if (path == "/api/v1/config") return request(USB_OP_PUT_CONFIG, &body);
        throw std::runtime_error("unsupported USB control path: " + path);
    }
    Json::Value post(const std::string &path, const Json::Value &body) override
    {
        (void)body; // the UDP port argument is meaningless on USB
        if (path == "/api/v1/stream/start") return request(USB_OP_STREAM_START, nullptr);
        if (path == "/api/v1/stream/stop") return request(USB_OP_STREAM_STOP, nullptr);
        throw std::runtime_error("unsupported USB control path: " + path);
    }

private:
    Json::Value request(uint32_t opcode, const Json::Value *body)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        std::string payload = body != nullptr ? jsonString(*body) : std::string();
        if (payload.size() > USB_CTRL_MAX_PAYLOAD) throw std::runtime_error("USB control payload too large");
        // Keep the request off exact packet-size multiples so the transfer
        // always terminates with a short packet.
        if ((USB_CTRL_HEADER_BYTES + payload.size()) % 512 == 0) payload.push_back(' ');
        const uint32_t sequence = ++_sequence;
        std::vector<uint8_t> out(USB_CTRL_HEADER_BYTES + payload.size());
        std::memcpy(out.data(), "IQRQ", 4);
        writeLe32(out.data() + 4, sequence);
        writeLe32(out.data() + 8, opcode);
        writeLe32(out.data() + 12, static_cast<uint32_t>(payload.size()));
        std::memcpy(out.data() + USB_CTRL_HEADER_BYTES, payload.data(), payload.size());
        int transferred = 0;
        int status = libusb_bulk_transfer(_usb->handle, USB_EP_CTRL_OUT, out.data(),
                                          static_cast<int>(out.size()), &transferred, USB_CTRL_TIMEOUT_MS);
        if (status != 0 || transferred != static_cast<int>(out.size())) {
            throw std::runtime_error(std::string("USB control write failed: ") + libusb_error_name(status));
        }
        std::vector<uint8_t> in(USB_CTRL_HEADER_BYTES + USB_CTRL_MAX_PAYLOAD + 64);
        status = libusb_bulk_transfer(_usb->handle, USB_EP_CTRL_IN, in.data(),
                                      static_cast<int>(in.size()), &transferred, USB_CTRL_TIMEOUT_MS);
        if (status != 0 || transferred < static_cast<int>(USB_CTRL_HEADER_BYTES)) {
            throw std::runtime_error(std::string("USB control read failed: ") + libusb_error_name(status));
        }
        if (std::memcmp(in.data(), "IQRS", 4) != 0 || le32(in.data() + 4) != sequence) {
            throw std::runtime_error("USB control response out of sync");
        }
        const uint32_t errorStatus = le32(in.data() + 8);
        const uint32_t payloadBytes = le32(in.data() + 12);
        if (USB_CTRL_HEADER_BYTES + payloadBytes > static_cast<std::size_t>(transferred)) {
            throw std::runtime_error("USB control response truncated");
        }
        const std::string text(reinterpret_cast<char *>(in.data()) + USB_CTRL_HEADER_BYTES, payloadBytes);
        if (errorStatus != 0) throw std::runtime_error("ESP-SDR USB control error: " + text);
        return text.empty() ? Json::Value(Json::objectValue) : parseJson(text);
    }

    static void writeLe32(uint8_t *p, uint32_t value)
    {
        p[0] = value & 0xff;
        p[1] = (value >> 8) & 0xff;
        p[2] = (value >> 16) & 0xff;
        p[3] = (value >> 24) & 0xff;
    }

    std::shared_ptr<UsbContext> _usb;
    std::mutex _mutex;
    uint32_t _sequence = 0;
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
    std::shared_ptr<UsbContext> usb; // non-null when streaming over USB
    std::vector<uint8_t> usbParseBuffer;
    std::atomic<int> usbActiveTransfers{0};
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
    std::atomic<unsigned> cycleTotal{1};
    std::atomic<unsigned> cycleStream{1};
};

class EspDevice final : public SoapySDR::Device {
public:
    explicit EspDevice(const SoapySDR::Kwargs &args):
        _host(valueOr(args, "host", "esp-sdr.local")),
        _httpPort(parseUnsigned(valueOr(args, "http_port", "80"), "http_port", 1, 65535)),
        _requestedUdpPort(parseUnsigned(valueOr(args, "udp_port", "0"), "udp_port", 0, 65535)),
        _rxBufferBytes(parseUnsigned(valueOr(args, "rx_buffer_bytes", "33554432"), "rx_buffer_bytes", 65536, 268435456)),
        _cycleTotal(parseUnsigned(valueOr(args, "cycle_total", "1"), "cycle_total", 1, STREAM_SELECTION_MAX)),
        _cycleStream(parseUnsigned(valueOr(args, "cycle_stream", "1"), "cycle_stream", 1, STREAM_SELECTION_MAX))
    {
        if (_cycleStream > _cycleTotal) throw std::runtime_error("cycle_stream must not exceed cycle_total");
        const std::string usbArg = valueOr(args, "usb", "");
        const std::string usbSerial = valueOr(args, "usb_serial", "");
        if ((!usbArg.empty() && usbArg != "0") || !usbSerial.empty()) {
            _usb = usbOpen(usbSerial);
            _control = std::make_unique<UsbControl>(_usb);
        } else {
            _control = std::make_unique<HttpControl>(_host, _httpPort);
        }
        _config = _control->get("/api/v1/config");
        if (!_config.isObject()) throw std::runtime_error("ESP-SDR returned an invalid configuration");
        const Json::Value status = _control->get("/api/v1/status");
        const Json::Value gain = status["manual_rx_gain"];
        if (gain.isObject() && gain["unit"].asString() == "dB") {
            _gainMin = gain.get("minimum", 0.0).asDouble();
            _gainMax = gain.get("maximum", 76.0).asDouble();
            _gainStep = gain.get("step", 1.0).asDouble();
        }
        applyDutyCycle(_cycleTotal, _cycleStream);
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
        if (_usb != nullptr) {
            return {{"vendor", "Espressif"}, {"hardware", "ESP32-S31 Function-CoreBoard"},
                    {"usb_serial", _usb->serial}, {"transport", "USB control / USB IQ"}};
        }
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
        patch["gain"]["rx_gain"] = static_cast<unsigned>(
            std::clamp(std::round(value), _gainMin, _gainMax));
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
        return {_gainMin, _gainMax, _gainStep};
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
        return supportedSampleRates();
    }
    SoapySDR::RangeList getSampleRateRange(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        SoapySDR::RangeList ranges;
        for (const double rate : supportedSampleRates()) ranges.emplace_back(rate, rate);
        return ranges;
    }

    SoapySDR::ArgInfoList getSettingInfo() const override
    {
        SoapySDR::ArgInfo total;
        total.key = "cycle_total";
        total.value = "1";
        total.name = "Duty cycle: total chunks";
        total.description = "Total number of chunks in one capture cycle";
        total.units = "chunks";
        total.type = SoapySDR::ArgInfo::INT;
        total.range = SoapySDR::Range(1, STREAM_SELECTION_MAX, 1);
        SoapySDR::ArgInfo streamed = total;
        streamed.key = "cycle_stream";
        streamed.name = "Duty cycle: streamed chunks";
        streamed.description = "Number of contiguous chunks streamed at the start of each capture cycle";
        return {total, streamed};
    }
    void writeSetting(const std::string &key, const std::string &value) override
    {
        unsigned total = _cycleTotal.load();
        unsigned streamed = _cycleStream.load();
        if (key == "cycle_total") {
            total = parseUnsigned(value, "cycle_total", 1, STREAM_SELECTION_MAX);
            streamed = std::min(streamed, total);
        } else if (key == "cycle_stream") {
            streamed = parseUnsigned(value, "cycle_stream", 1, STREAM_SELECTION_MAX);
            if (streamed > total) throw std::runtime_error("cycle_stream must not exceed cycle_total");
        } else {
            throw std::runtime_error("unknown setting: " + key);
        }
        applyDutyCycle(total, streamed);
    }
    std::string readSetting(const std::string &key) const override
    {
        if (key == "cycle_total") return std::to_string(_cycleTotal.load());
        if (key == "cycle_stream") return std::to_string(_cycleStream.load());
        throw std::runtime_error("unknown setting: " + key);
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
        if (_usb != nullptr) {
            state->usb = _usb;
            _stream = state.release();
            return reinterpret_cast<SoapySDR::Stream *>(_stream);
        }
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
        const unsigned total = _cycleTotal.load();
        const unsigned streamed = _cycleStream.load();
        patch["trigger"]["trigger_config"] = intervalTrigger(total, streamed);
        applyPatch(patch);
        state->cycleTotal = total;
        state->cycleStream = streamed;
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
        state->worker = std::thread(state->usb != nullptr ? &EspDevice::usbReceiveLoop
                                                          : &EspDevice::receiveLoop, state);
        Json::Value body;
        body["port"] = state->port;
        try {
            _control->post("/api/v1/stream/start", body);
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
        try { _control->post("/api/v1/stream/stop", Json::Value(Json::objectValue)); }
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
        for (unsigned decimation = 1; decimation <= ADC_DECIMATION_MAX; ++decimation) {
            if (std::abs(rate - double(ADC_CLOCK_HZ) / decimation) < 1) return decimation;
        }
        throw std::runtime_error("sample rate must be 80 MSa/s divided by an integer from 1 to 10");
    }
    static std::vector<double> supportedSampleRates()
    {
        std::vector<double> rates;
        for (unsigned decimation = ADC_DECIMATION_MAX; decimation != 0; --decimation) {
            rates.push_back(double(ADC_CLOCK_HZ) / decimation);
        }
        return rates;
    }
    void applyDutyCycle(unsigned total, unsigned streamed)
    {
        if (total == 0 || total > STREAM_SELECTION_MAX || streamed == 0 || streamed > total) {
            throw std::runtime_error("duty cycle requires 1 <= cycle_stream <= cycle_total <= 1000000");
        }
        Json::Value patch;
        patch["trigger"]["trigger_mode"] = 0;
        patch["trigger"]["trigger_config"] = intervalTrigger(total, streamed);
        applyPatch(patch);
        _cycleTotal = total;
        _cycleStream = streamed;
        if (_stream != nullptr) {
            _stream->cycleTotal = total;
            _stream->cycleStream = streamed;
        }
    }
    static uint32_t nextSelectedSource(uint32_t source, unsigned total, unsigned streamed)
    {
        const unsigned position = source % total;
        return position + 1u < streamed ? source + 1u : source + (total - position);
    }
    static uint32_t selectedCount(uint32_t begin, uint32_t end, unsigned total, unsigned streamed)
    {
        if (end <= begin) return 0;
        const auto before = [total, streamed](uint32_t value) -> uint64_t {
            const uint64_t cycles = value / total;
            const unsigned remainder = value % total;
            return cycles * streamed + std::min(remainder, streamed);
        };
        return static_cast<uint32_t>(before(end) - before(begin));
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
            _control->put("/api/v1/config", patch);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (std::chrono::steady_clock::now() < deadline) {
                const Json::Value status = _control->get("/api/v1/status");
                if (!status.get("config_applying", false).asBool()) {
                    Json::Value applied = _control->get("/api/v1/config");
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
        const unsigned total = state->cycleTotal.load();
        const unsigned streamed = state->cycleStream.load();
        if (state->haveExpectedSource) {
            if (source < state->expectedSource) {
                // Applying RF configuration restarts capture at a low source
                // index without changing the UDP epoch. Treat only a small
                // backward step as reordering; otherwise begin a new capture
                // generation instead of rejecting the restarted stream forever.
                if (state->expectedSource - source <= std::max(8u, total * 2u)) return;
                state->queue.clear();
                state->captureRestarts++;
                if (!suppressContinuity) state->overflowPending = true;
            }
            if (source > state->expectedSource) {
                if (!suppressContinuity) {
                    state->lostChunks += selectedCount(state->expectedSource, source, total, streamed);
                    state->overflowPending = true;
                }
            }
        }
        state->expectedSource = nextSelectedSource(source, total, streamed);
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
            const unsigned reorderWindow = std::max(8u, state->cycleTotal.load() * 2u);
            if (state->haveExpectedSource && header.sourceChunk < state->expectedSource &&
                state->expectedSource - header.sourceChunk > reorderWindow) {
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

    // USB stream: a reliable byte stream of self-delimiting messages, each a
    // 52-byte IQU1 header (fragment_count == 1) followed by the whole frame.
    static void usbConsume(StreamState *state)
    {
        auto &buffer = state->usbParseBuffer;
        std::size_t pos = 0;
        while (buffer.size() - pos >= UDP_HEADER_BYTES) {
            const uint8_t *p = buffer.data() + pos;
            if (std::memcmp(p, "IQU1", 4) != 0 ||
                le32(p + 48) != static_cast<uint32_t>(crc32(0, p, 48))) {
                ++pos; // resync byte-wise on the next valid header
                state->invalidDatagrams++;
                continue;
            }
            const std::size_t total = UDP_HEADER_BYTES + le16(p + 36);
            if (buffer.size() - pos < total) break;
            handleDatagram(state, p, total);
            pos += total;
        }
        buffer.erase(buffer.begin(), buffer.begin() + pos);
    }

    static void usbStreamCallback(libusb_transfer *transfer)
    {
        auto *state = static_cast<StreamState *>(transfer->user_data);
        bool resubmit = false;
        if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
            state->usbParseBuffer.insert(state->usbParseBuffer.end(), transfer->buffer,
                                         transfer->buffer + transfer->actual_length);
            usbConsume(state);
            resubmit = true;
        }
        if (resubmit && !state->stop && libusb_submit_transfer(transfer) == 0) return;
        state->usbActiveTransfers--;
    }

    static void usbReceiveLoop(StreamState *state)
    {
        std::array<libusb_transfer *, USB_STREAM_TRANSFERS> transfers{};
        std::vector<std::vector<uint8_t>> buffers(USB_STREAM_TRANSFERS,
                                                  std::vector<uint8_t>(USB_STREAM_TRANSFER_BYTES));
        for (int i = 0; i < USB_STREAM_TRANSFERS; ++i) {
            transfers[i] = libusb_alloc_transfer(0);
            libusb_fill_bulk_transfer(transfers[i], state->usb->handle, USB_EP_STREAM_IN,
                                      buffers[i].data(), static_cast<int>(buffers[i].size()),
                                      &EspDevice::usbStreamCallback, state, 0);
            if (libusb_submit_transfer(transfers[i]) == 0) state->usbActiveTransfers++;
        }
        while (!state->stop) {
            timeval tv{0, 100000};
            libusb_handle_events_timeout(state->usb->context, &tv);
        }
        for (int i = 0; i < USB_STREAM_TRANSFERS; ++i) {
            if (transfers[i] != nullptr) libusb_cancel_transfer(transfers[i]);
        }
        while (state->usbActiveTransfers > 0) {
            timeval tv{0, 100000};
            libusb_handle_events_timeout(state->usb->context, &tv);
        }
        for (auto *transfer : transfers) {
            if (transfer != nullptr) libusb_free_transfer(transfer);
        }
        state->usbParseBuffer.clear();
    }

    std::string _host;
    unsigned _httpPort;
    unsigned _requestedUdpPort;
    unsigned _rxBufferBytes;
    std::atomic<unsigned> _cycleTotal;
    std::atomic<unsigned> _cycleStream;
    std::shared_ptr<UsbContext> _usb;
    std::unique_ptr<Control> _control;
    std::mutex _controlMutex;
    mutable std::mutex _configMutex;
    Json::Value _config;
    double _gainMin = 0.0;
    double _gainMax = 76.0;
    double _gainStep = 1.0;
    StreamState *_stream = nullptr;
};

std::vector<std::string> usbEnumerateSerials()
{
    std::vector<std::string> serials;
    libusb_context *context = nullptr;
    if (libusb_init(&context) != 0) return serials;
    libusb_device **list = nullptr;
    const ssize_t count = libusb_get_device_list(context, &list);
    for (ssize_t i = 0; i < count; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != 0) continue;
        if (desc.idVendor != USB_VID || desc.idProduct != USB_PID) continue;
        libusb_device_handle *handle = nullptr;
        if (libusb_open(list[i], &handle) != 0) continue;
        if (usbStringDescriptor(handle, desc.iProduct) == USB_PRODUCT) {
            serials.push_back(usbStringDescriptor(handle, desc.iSerialNumber));
        }
        libusb_close(handle);
    }
    if (list != nullptr) libusb_free_device_list(list, 1);
    libusb_exit(context);
    return serials;
}

SoapySDR::KwargsList findEspSdr(const SoapySDR::Kwargs &args)
{
    const auto driver = args.find("driver");
    if (driver != args.end() && driver->second != "espsdr") return {};

    SoapySDR::KwargsList results;
    const auto usbArg = args.find("usb");
    const auto usbSerialArg = args.find("usb_serial");
    const bool wantUsb = (usbArg != args.end() && usbArg->second != "0") || usbSerialArg != args.end();

    for (const std::string &serial : usbEnumerateSerials()) {
        if (usbSerialArg != args.end() && usbSerialArg->second != serial) continue;
        SoapySDR::Kwargs result = args;
        result["driver"] = "espsdr";
        result["usb"] = "1";
        result["usb_serial"] = serial;
        result["label"] = "ESP-SDR USB (" + serial + ")";
        results.push_back(std::move(result));
    }
    if (wantUsb) return results;

    SoapySDR::Kwargs result = args;
    result["driver"] = "espsdr";
    if (result.count("host") == 0) result["host"] = "esp-sdr.local";
    const unsigned port = result.count("http_port") ? static_cast<unsigned>(std::stoul(result["http_port"])) : 80;
    try {
        HttpClient http(result["host"], port);
        const Json::Value status = http.get("/api/v1/status");
        result["label"] = "ESP-SDR (" + result["host"] + ")";
        if (status.isMember("ipv4")) result["ipv4"] = status["ipv4"].asString();
        results.push_back(std::move(result));
    } catch (const std::exception &error) {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyESPSDR discovery at %s failed: %s", result["host"].c_str(), error.what());
    }
    return results;
}

SoapySDR::Device *makeEspSdr(const SoapySDR::Kwargs &args) { return new EspDevice(args); }
static SoapySDR::Registry registerEspSdr("espsdr", &findEspSdr, &makeEspSdr, SOAPY_SDR_ABI_VERSION);

} // namespace
