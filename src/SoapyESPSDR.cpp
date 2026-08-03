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
#include <complex>
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
// Compressed IQC8 frame: same header, interleaved int8 I/Q pairs (top 8 of
// 10 sample bits), zero CRC field. Ethernet uses this to stay below the link
// ceiling; USB may return full IQC1 and convert to the requested host format.
constexpr std::size_t IQ8_FRAME_BYTES = IQ_HEADER_BYTES + IQ_SAMPLES * 2 + 4;
constexpr std::size_t REAL8_SAMPLES = IQ_SAMPLES * 4;
constexpr std::size_t REAL8_FRAME_BYTES = IQ_HEADER_BYTES + REAL8_SAMPLES + 4;
constexpr std::size_t MAX_BLOCK_SAMPLES = REAL8_SAMPLES / 2;
constexpr double ETHERNET_SAMPLE_RATE = 16e6;
constexpr std::size_t MAX_FRAME_BYTES = 64 * 1024;
// Absorb host-side scheduling stalls without discarding RF frames. At 16 MS/s
// this provides about 260 ms of elasticity.
constexpr std::size_t MAX_QUEUE_BLOCKS = 4096;
constexpr uint32_t UDP_VERSION = 1;
constexpr unsigned STREAM_SELECTION_MAX = 1'000'000;
constexpr uint32_t S31_ADC_SOURCE_MUX_MASK = 0x0000000fu;
constexpr std::size_t REAL_TIMING_TAPS = 25;
// Ignore the first 8 ms after arming (modem AGC/startup transient), then use
// the following 32 ms for the one-time interleaver calibration.  Every raw
// frame remains buffered and is released in order after calibration.
constexpr std::size_t REAL_TIMING_WARMUP_FRAMES = 64;
constexpr std::size_t REAL_TIMING_MEASURE_FRAMES = 256;
constexpr std::size_t REAL_TIMING_CALIBRATION_FRAMES =
    REAL_TIMING_WARMUP_FRAMES + REAL_TIMING_MEASURE_FRAMES;

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
constexpr int USB_STREAM_TRANSFERS = 8;
constexpr std::size_t USB_STREAM_TRANSFER_BYTES = 256 * 1024;

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
        // The UDP port argument is meaningless on USB, but the stream-start
        // body also carries the wire-format selection.
        if (path == "/api/v1/stream/start") return request(USB_OP_STREAM_START, &body);
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
    std::array<int16_t, MAX_BLOCK_SAMPLES * 2> iq{};
    std::size_t offset = 0;
    std::size_t samples = IQ_SAMPLES;
};

// 95-tap equiripple half-band filter: 7.2 MHz passband, 8.8 MHz stopband at
// 32 MS/s, 0.0015 dB ripple and >81 dB image rejection. Every other tap is
// zero, and symmetry reduces each 16 MS/s complex output to 24 real MACs.
constexpr std::array<float, 24> REAL_HALF_BAND = {{
    -8.92005200703e-05f, 0.00013469961315f,
    -0.000231858370999f, 0.000372127242862f,
    -0.000565968039619f, 0.000827792766952f,
    -0.0011717411679f, 0.00161621326943f,
    -0.00217988907681f, 0.00288658089657f,
    -0.00376181252064f, 0.00483850628388f,
    -0.00615511356419f, 0.00776369912039f,
    -0.00973238651044f, 0.0121608180731f,
    -0.0151968527782f, 0.0190806965405f,
    -0.0242249383936f, 0.0314146099535f,
    -0.042335421175f, 0.0613869749566f,
    -0.104724206642f, 0.317848019165f,
}};

// Periodically time-varying fractional-delay correction for the PARLIO
// diagnostic byte's two sampling phases. Pluto calibration measured the phase
// separation as 0.2125 full-rate samples. The opposite-phase FIR is this array
// reversed; which one applies to even samples depends on the arbitrary PARLIO
// stream start phase and is detected from the first eight frames.
constexpr std::array<float, REAL_TIMING_TAPS> REAL_TIMING_PHASE = {{
    0.00210546176977f, -0.00395768309485f, 0.00478342085786f,
    -0.00685873155397f, 0.00828749370302f, -0.0113467772303f,
    0.0137575109676f, -0.0190021359772f, 0.0237643068762f,
    -0.0358857837326f, 0.0506556060547f, -0.122746636936f,
    1.03992338111f, 0.0994251540500f, -0.0573636365137f,
    0.0336953104240f, -0.0260232555258f, 0.0184589876296f,
    -0.0151582839739f, 0.0112245943668f, -0.00936390453089f,
    0.00686844744574f, -0.00573979308791f, 0.00396546177867f,
    -0.00346851487415f,
}};

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
    std::array<float, 190> realHistory{};
    std::array<float, 190> imagHistory{};
    std::size_t historyPosition = 94;
    unsigned mixerPhase = 2;
    std::array<float, REAL_TIMING_TAPS * 2> timingHistory{};
    std::size_t timingHistoryPosition = REAL_TIMING_TAPS - 1;
    std::atomic<int> timingSkewSign{0};
    std::array<float, 2> timingOffset{};
    float timingCoefficientScale = 1.0f;
    float timingOddGain = 1.0f;
    std::atomic<int> timingSkewPpm{0};
    std::atomic<int> timingOddGainPpm{1'000'000};
    std::vector<std::array<int8_t, REAL8_SAMPLES>> timingCalibration;
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
        const auto correction = args.find("frequency_correction_ppm");
        if (correction != args.end()) {
            setFrequencyCorrection(SOAPY_SDR_RX, 0,
                                   parseDouble(correction->second,
                                               "frequency_correction_ppm",
                                               -100.0, 100.0));
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

    bool hasFrequencyCorrection(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return true;
    }
    void setFrequencyCorrection(const int direction, const std::size_t channel,
                                const double value) override
    {
        checkRx(direction, channel);
        if (!std::isfinite(value) || value < -100.0 || value > 100.0)
            throw std::runtime_error("frequency correction must be -100..100 ppm");
        Json::Value patch;
        patch["radio"]["frequency_correction_ppb"] =
            Json::Int64(std::llround(value * 1000.0));
        applyPatch(patch);
    }
    double getFrequencyCorrection(const int direction,
                                  const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return configInt64("radio", "frequency_correction_ppb", 0) / 1000.0;
    }

    void setSampleRate(const int direction, const std::size_t channel, const double rate) override
    {
        checkRx(direction, channel);
        const double supported = _usb == nullptr ? ETHERNET_SAMPLE_RATE : 2e6;
        if (std::abs(rate - supported) >= 1)
            throw std::runtime_error("sample rate must be " +
                                     std::to_string(supported / 1e6) +
                                     " MSa/s for this transport");
        Json::Value patch;
        patch["iq_engine"]["adc_decimation"] = _usb == nullptr ? 1 : 2;
        patch["rx_filter"]["rx_filter_override"] = _usb == nullptr ? 62 : 0;
        applyPatch(patch);
    }
    double getSampleRate(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return _usb == nullptr ? ETHERNET_SAMPLE_RATE : 2e6;
    }
    std::vector<double> listSampleRates(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        return {getSampleRate(direction, channel)};
    }
    SoapySDR::RangeList getSampleRateRange(const int direction, const std::size_t channel) const override
    {
        checkRx(direction, channel);
        SoapySDR::RangeList ranges;
        const double rate = getSampleRate(direction, channel);
        ranges.emplace_back(rate, rate);
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

        SoapySDR::ArgInfo correction;
        correction.key = "frequency_correction_ppm";
        correction.value = "0";
        correction.name = "Frequency correction";
        correction.description = "Board reference correction; positive means the ESP receiver LO runs high";
        correction.units = "ppm";
        correction.type = SoapySDR::ArgInfo::FLOAT;
        correction.range = SoapySDR::Range(-100, 100);

        SoapySDR::ArgInfo filterOverride;
        filterOverride.key = "rx_filter_override";
        filterOverride.value = "0";
        filterOverride.name = "Expert RX filter override";
        filterOverride.description = "0=calibrated complex path; 62=32 MS/s real PARLIO plus host analytic conversion";
        filterOverride.type = SoapySDR::ArgInfo::INT;
        filterOverride.range = SoapySDR::Range(0, 62, 1);

        SoapySDR::ArgInfo filterMode;
        filterMode.key = "rx_filter_mode";
        filterMode.value = "32";
        filterMode.name = "Expert RX filter mode";
        filterMode.description = "Expert mode; BT probes use 0..31 for byte pairs and 32 for vendor BT packing";
        filterMode.type = SoapySDR::ArgInfo::INT;
        filterMode.range = SoapySDR::Range(0, 32, 1);

        SoapySDR::ArgInfo filterDcap;
        filterDcap.key = "rx_filter_dcap";
        filterDcap.value = "60";
        filterDcap.name = "Expert RX filter DCap";
        filterDcap.description = "Raw 6-bit Wi-Fi RX analog filter capacitor-DAC code";
        filterDcap.type = SoapySDR::ArgInfo::INT;
        filterDcap.range = SoapySDR::Range(0, 63, 1);

        SoapySDR::ArgInfo adcSource;
        adcSource.key = "adc_source_sel";
        adcSource.value = "3";
        adcSource.name = "Expert ADC dump source";
        adcSource.description = "Raw S31 ADC dump source mux (low four bits)";
        adcSource.type = SoapySDR::ArgInfo::INT;
        adcSource.range = SoapySDR::Range(0, 15, 1);

        SoapySDR::ArgInfo loopback;
        loopback.key = "diag_loopback";
        loopback.value = "0";
        loopback.name = "Diagnostic internal loopback";
        loopback.type = SoapySDR::ArgInfo::BOOL;

        SoapySDR::ArgInfo toneEnable = loopback;
        toneEnable.key = "diag_tx_tone_enable";
        toneEnable.name = "Diagnostic TX tone";

        SoapySDR::ArgInfo toneStep;
        toneStep.key = "diag_tx_tone_step";
        toneStep.value = "16";
        toneStep.name = "Diagnostic TX tone step";
        toneStep.type = SoapySDR::ArgInfo::INT;
        toneStep.range = SoapySDR::Range(0, 100, 1);

        SoapySDR::ArgInfo loopTxGain;
        loopTxGain.key = "diag_loopback_tx_gain";
        loopTxGain.value = "124";
        loopTxGain.name = "Diagnostic loopback TX gain";
        loopTxGain.type = SoapySDR::ArgInfo::INT;
        loopTxGain.range = SoapySDR::Range(0, 255, 1);

        SoapySDR::ArgInfo loopRxGain = loopTxGain;
        loopRxGain.key = "diag_loopback_rx_gain";
        loopRxGain.value = "115";
        loopRxGain.name = "Diagnostic loopback RX gain";
        loopRxGain.range = SoapySDR::Range(0, 127, 1);

        SoapySDR::ArgInfo loopBbGain = loopRxGain;
        loopBbGain.key = "diag_loopback_bb_gain";
        loopBbGain.value = "63";
        loopBbGain.name = "Diagnostic loopback BB gain";

        return {total, streamed, correction, filterOverride, filterMode, filterDcap, adcSource,
                loopback, toneEnable, toneStep, loopTxGain, loopRxGain, loopBbGain};
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
        } else if (key == "frequency_correction_ppm") {
            setFrequencyCorrection(SOAPY_SDR_RX, 0,
                                   parseDouble(value, "frequency_correction_ppm",
                                               -100.0, 100.0));
            return;
        } else if (key == "rx_filter_override") {
            Json::Value patch;
            patch["rx_filter"]["rx_filter_override"] = parseUnsigned(value, "rx_filter_override", 0, 62);
            applyPatch(patch);
            return;
        } else if (key == "rx_filter_mode") {
            Json::Value patch;
            patch["rx_filter"]["rx_filter_mode"] = parseUnsigned(value, "rx_filter_mode", 0, 32);
            applyPatch(patch);
            return;
        } else if (key == "rx_filter_dcap") {
            Json::Value patch;
            patch["rx_filter"]["rx_filter_dcap"] = parseUnsigned(value, "rx_filter_dcap", 0, 63);
            applyPatch(patch);
            return;
        } else if (key == "adc_source_sel") {
            Json::Value patch;
            const unsigned source = parseUnsigned(value, "adc_source_sel", 0, 15);
            const uint32_t current = configUInt("iq_engine", "adc_source_sel", 3);
            patch["iq_engine"]["adc_source_sel"] =
                (current & ~S31_ADC_SOURCE_MUX_MASK) | source;
            applyPatch(patch);
            return;
        } else if (key == "diag_loopback") {
            Json::Value patch;
            patch["loopback"]["loopback"] = parseUnsigned(value, "diag_loopback", 0, 1);
            applyPatch(patch);
            return;
        } else if (key == "diag_tx_tone_enable") {
            Json::Value patch;
            patch["tx"]["tx_tone_enable"] = parseUnsigned(value, "diag_tx_tone_enable", 0, 1);
            applyPatch(patch);
            return;
        } else if (key == "diag_tx_tone_step") {
            Json::Value patch;
            patch["tx"]["tx_tone0_step"] = parseUnsigned(value, "diag_tx_tone_step", 0, 100);
            applyPatch(patch);
            return;
        } else if (key == "diag_loopback_tx_gain") {
            Json::Value patch;
            patch["loopback"]["loopback_tx_gain"] = parseUnsigned(value, "diag_loopback_tx_gain", 0, 255);
            applyPatch(patch);
            return;
        } else if (key == "diag_loopback_rx_gain") {
            Json::Value patch;
            patch["loopback"]["loopback_rx_gain"] = parseUnsigned(value, "diag_loopback_rx_gain", 0, 127);
            applyPatch(patch);
            return;
        } else if (key == "diag_loopback_bb_gain") {
            Json::Value patch;
            patch["loopback"]["loopback_bb_gain"] = parseUnsigned(value, "diag_loopback_bb_gain", 0, 127);
            applyPatch(patch);
            return;
        } else {
            throw std::runtime_error("unknown setting: " + key);
        }
        applyDutyCycle(total, streamed);
    }
    std::string readSetting(const std::string &key) const override
    {
        if (key == "cycle_total") return std::to_string(_cycleTotal.load());
        if (key == "cycle_stream") return std::to_string(_cycleStream.load());
        if (key == "frequency_correction_ppm") return std::to_string(
            getFrequencyCorrection(SOAPY_SDR_RX, 0));
        if (key == "rx_filter_override") return std::to_string(configUInt("rx_filter", "rx_filter_override", 0));
        if (key == "rx_filter_mode") return std::to_string(configUInt("rx_filter", "rx_filter_mode", 16));
        if (key == "rx_filter_dcap") return std::to_string(configUInt("rx_filter", "rx_filter_dcap", 60));
        if (key == "adc_source_sel") return std::to_string(configUInt("iq_engine", "adc_source_sel", 3) & 15u);
        if (key == "diag_loopback") return std::to_string(configUInt("loopback", "loopback", 0));
        if (key == "diag_tx_tone_enable") return std::to_string(configUInt("tx", "tx_tone_enable", 0));
        if (key == "diag_tx_tone_step") return std::to_string(configUInt("tx", "tx_tone0_step", 16));
        if (key == "diag_loopback_tx_gain") return std::to_string(configUInt("loopback", "loopback_tx_gain", 124));
        if (key == "diag_loopback_rx_gain") return std::to_string(configUInt("loopback", "loopback_rx_gain", 115));
        if (key == "diag_loopback_bb_gain") return std::to_string(configUInt("loopback", "loopback_bb_gain", 63));
        throw std::runtime_error("unknown setting: " + key);
    }

    void setBandwidth(const int direction, const std::size_t channel, const double bandwidth) override
    {
        checkRx(direction, channel);
        unsigned mhz = 0;
        if (bandwidth != 0) {
            mhz = static_cast<unsigned>(std::llround(bandwidth / 1e6));
            if (mhz < 13 || mhz > 54)
                throw std::runtime_error("bandwidth must be 0/open or 13-54 MHz");
        }
        Json::Value patch;
        patch["bandwidth"]["bw_mhz"] = 20;
        patch["bandwidth"]["second_chan"] = 0;
        patch["rx_filter"]["filter_bw_mhz"] = mhz;
        patch["rx_filter"]["rx_filter_override"] = _usb == nullptr ? 62 : 0;
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
        return {SOAPY_SDR_CS16, SOAPY_SDR_CF32, SOAPY_SDR_CS8};
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
        if (format != SOAPY_SDR_CS8 && format != SOAPY_SDR_CS16 &&
            format != SOAPY_SDR_CF32) {
            throw std::runtime_error("supported formats are CS16, CF32, and CS8");
        }
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
        (void)checkedStream(stream);
        /* IQR8 carries 4096 real samples and produces 2048 complex samples
         * on both transports. Advertising that full host-DSP block also
         * remains valid when talking to older IQC1 USB firmware. */
        return MAX_BLOCK_SAMPLES;
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
        patch["iq_engine"]["adc_decimation"] = state->usb == nullptr ? 1 : 2;
        patch["rx_filter"]["rx_filter_override"] = state->usb == nullptr ? 62 : 0;
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
            resetRealDsp(state);
        }
        state->stop = false;
        state->active = true;
        state->worker = std::thread(state->usb != nullptr ? &EspDevice::usbReceiveLoop
                                                          : &EspDevice::receiveLoop, state);
        Json::Value body;
        body["port"] = state->port;
        // Current Ethernet and USB firmware emit IQR8; the requested value
        // remains compatible with older USB firmware that emitted IQC1.
        body["stream_format"] = state->usb == nullptr ? 1 : 0;
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
            const std::size_t count = std::min(numElems - produced,
                                               block.samples - block.offset);
            if (state->format == SOAPY_SDR_CS16) {
                auto *output = static_cast<int16_t *>(buffers[0]);
                std::memcpy(output + produced * 2, block.iq.data() + block.offset * 2, count * 2 * sizeof(int16_t));
            } else if (state->format == SOAPY_SDR_CS8) {
                auto *output = static_cast<int8_t *>(buffers[0]);
                for (std::size_t i = 0; i < count * 2; ++i) output[produced * 2 + i] = int8_t(block.iq[block.offset * 2 + i] / 4);
            } else {
                auto *output = static_cast<float *>(buffers[0]);
                for (std::size_t i = 0; i < count * 2; ++i) output[produced * 2 + i] = block.iq[block.offset * 2 + i] / 512.0f;
            }
            produced += count;
            block.offset += count;
            if (block.offset == block.samples) state->queue.pop_front();
        }
        return static_cast<int>(produced);
    }

    std::vector<std::string> listSensors() const override
    {
        return {"datagrams", "invalid_datagrams", "completed_frames", "lost_chunks",
                "firmware_drops", "queue_drops", "capture_restarts",
                "timing_skew_sign", "timing_skew_ppm", "timing_odd_gain_ppm",
                "adc_dump_cfg", "adc_dump_mode"};
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
        if (key == "timing_skew_sign") return std::to_string(state->timingSkewSign.load());
        if (key == "timing_skew_ppm") return std::to_string(state->timingSkewPpm.load());
        if (key == "timing_odd_gain_ppm") return std::to_string(state->timingOddGainPpm.load());
        if (key == "adc_dump_cfg" || key == "adc_dump_mode") {
            const Json::Value status = _control->get("/api/v1/status");
            return std::to_string(status[key].asUInt());
        }
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
    static double parseDouble(const std::string &text, const char *name,
                              double minimum, double maximum)
    {
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(value) ||
            value < minimum || value > maximum)
            throw std::runtime_error(std::string(name) + " is out of range");
        return value;
    }
    static void checkRx(const int direction, const std::size_t channel)
    {
        if (direction != SOAPY_SDR_RX || channel != 0) throw std::runtime_error("SoapyESPSDR supports RX channel 0 only");
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
    int64_t configInt64(const char *group, const char *key, int64_t fallback) const
    {
        std::lock_guard<std::mutex> lock(_configMutex);
        const Json::Value &value = _config[group][key];
        return value.isNumeric() ? value.asInt64() : fallback;
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
    static void resetRealDsp(StreamState *state)
    {
        state->realHistory.fill(0.0f);
        state->imagHistory.fill(0.0f);
        state->historyPosition = 94;
        // Keep the fractional-delay FIR's causalized group delay in the Fs/4
        // mixer phase. This preserves one output for every input pair without
        // a frequency or conjugation error.
        state->mixerPhase =
            (4u - unsigned((REAL_TIMING_TAPS / 2) & 3u)) & 3u;
        state->timingHistory.fill(0.0f);
        state->timingHistoryPosition = REAL_TIMING_TAPS - 1;
        state->timingSkewSign = 0;
        state->timingOffset.fill(0.0f);
        /* The 4 MHz USB capture clock is synchronous with the ADC update
         * cadence and needs no alternating fractional-delay correction. The
         * 32 MHz Ethernet clock crosses the 80 MHz ADC cadence and retains
         * the measured conservative correction until calibration refines it. */
        state->timingCoefficientScale = state->usb != nullptr ? 0.0f : 1.0f;
        state->timingOddGain = 1.0f;
        state->timingSkewPpm = 0;
        state->timingOddGainPpm = 1'000'000;
        state->timingCalibration.clear();
        state->timingCalibration.reserve(REAL_TIMING_CALIBRATION_FRAMES);
    }
    static void calibrationFft(std::vector<std::complex<float>> &data)
    {
        const std::size_t size = data.size();
        for (std::size_t i = 1, j = 0; i < size; ++i) {
            std::size_t bit = size >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) std::swap(data[i], data[j]);
        }
        for (std::size_t length = 2; length <= size; length <<= 1) {
            const float angle = -2.0f * float(M_PI) / float(length);
            const std::complex<float> step(std::cos(angle), std::sin(angle));
            for (std::size_t start = 0; start < size; start += length) {
                std::complex<float> rotation(1.0f, 0.0f);
                for (std::size_t offset = 0; offset < length / 2; ++offset) {
                    const auto even = data[start + offset];
                    const auto odd = data[start + offset + length / 2] * rotation;
                    data[start + offset] = even + odd;
                    data[start + offset + length / 2] = even - odd;
                    rotation *= step;
                }
            }
        }
    }
    static int detectTimingSkew(StreamState *state)
    {
        double evenMean = 0.0;
        double oddMean = 0.0;
        std::size_t pairs = 0;
        for (std::size_t frameIndex = REAL_TIMING_WARMUP_FRAMES;
             frameIndex < state->timingCalibration.size(); ++frameIndex) {
            const auto &frame = state->timingCalibration[frameIndex];
            for (std::size_t i = 0; i < REAL8_SAMPLES; i += 2) {
                evenMean += frame[i];
                oddMean += frame[i + 1];
                ++pairs;
            }
        }
        if (pairs == 0) return 1;
        evenMean /= pairs;
        oddMean /= pairs;
        state->timingOffset[0] = static_cast<float>(evenMean);
        state->timingOffset[1] = static_cast<float>(oddMean);
        double metric = 0.0;
        std::size_t pairIndex = 0;
        for (std::size_t frameIndex = REAL_TIMING_WARMUP_FRAMES;
             frameIndex < state->timingCalibration.size(); ++frameIndex) {
            const auto &frame = state->timingCalibration[frameIndex];
            for (std::size_t i = 0; i < REAL8_SAMPLES; i += 2, ++pairIndex) {
                if (pairIndex + 1 >= pairs) break;
                const double even = frame[i] - evenMean;
                const double odd = frame[i + 1] - oddMean;
                double nextEven;
                if (i + 2 < REAL8_SAMPLES) {
                    nextEven = frame[i + 2] - evenMean;
                } else {
                    if (frameIndex + 1 < state->timingCalibration.size()) {
                        nextEven =
                            state->timingCalibration[frameIndex + 1][0] -
                            evenMean;
                    } else {
                        break;
                    }
                }
                metric += odd * nextEven - even * odd;
            }
        }
        int sign = metric >= 0.0 ? 1 : -1;
        state->timingSkewPpm = static_cast<int>(std::lround(
            sign * state->timingCoefficientScale * 212500.0f));

        // A dominant RF line lets us minimize the actual Fs/2-f image.  Do
        // not estimate the mismatch from the two decimated phase streams:
        // each is only 16 MS/s, so a line above 8 MHz aliases onto its own
        // conjugate and biases that estimate.  Instead, measure the desired
        // and image phasors in the full-rate spectrum and evaluate the exact
        // two-periodic FIR response over a small (skew, gain) grid.
        std::vector<std::complex<float>> spectrum;
        spectrum.reserve(pairs * 2);
        std::size_t sampleIndex = 0;
        const std::size_t calibrationSamples = pairs * 2;
        for (std::size_t frameIndex = REAL_TIMING_WARMUP_FRAMES;
             frameIndex < state->timingCalibration.size(); ++frameIndex) {
            const auto &frame = state->timingCalibration[frameIndex];
            for (std::size_t i = 0; i < REAL8_SAMPLES; ++i) {
                const double mean = (i & 1u) ? oddMean : evenMean;
                const float window = 0.5f - 0.5f * std::cos(
                    2.0f * float(M_PI) * float(sampleIndex) /
                    float(calibrationSamples - 1));
                spectrum.emplace_back(float(frame[i] - mean) * window, 0.0f);
                ++sampleIndex;
            }
        }
        calibrationFft(spectrum);
        const std::size_t firstBin = spectrum.size() / 64;
        const std::size_t lastBin = spectrum.size() / 2 - firstBin;
        std::size_t peakBin = firstBin;
        float peakPower = 0.0f;
        std::vector<float> powers;
        powers.reserve(lastBin - firstBin);
        for (std::size_t bin = firstBin; bin < lastBin; ++bin) {
            const float power = std::norm(spectrum[bin]);
            powers.push_back(power);
            if (power > peakPower) {
                peakPower = power;
                peakBin = bin;
            }
        }
        if (!powers.empty()) {
            auto middle = powers.begin() + powers.size() / 2;
            std::nth_element(powers.begin(), middle, powers.end());
            const float medianPower = std::max(*middle, 1e-20f);
            std::size_t strongClusters = 0;
            bool inStrongCluster = false;
            for (std::size_t bin = firstBin; bin < lastBin; ++bin) {
                const bool strong = std::norm(spectrum[bin]) >
                                    peakPower / 15.8489f; // peak - 12 dB
                if (strong && !inStrongCluster) ++strongClusters;
                inStrongCluster = strong;
            }
            // Internal modem spurs can exceed 15 dB with no antenna signal;
            // require a genuinely isolated calibration carrier before
            // adapting away from the conservative fixed correction. The
            // cluster-count check prevents an OFDM/multitone signal from
            // being mistaken for a calibration tone and equalized at one bin.
            if (peakPower > medianPower * 1000.0f && // 30 dB above floor
                strongClusters <= 4) {
                const std::size_t size = spectrum.size();
                const double left = std::log(std::max(
                    std::abs(spectrum[peakBin - 1]), 1e-20f));
                const double center = std::log(std::max(
                    std::abs(spectrum[peakBin]), 1e-20f));
                const double right = std::log(std::max(
                    std::abs(spectrum[peakBin + 1]), 1e-20f));
                const double curvature = left - 2.0 * center + right;
                const double fractionalBin = std::abs(curvature) > 1e-12
                    ? std::clamp(0.5 * (left - right) / curvature, -0.5, 0.5)
                    : 0.0;
                const double omega = 2.0 * M_PI *
                    (double(peakBin) + fractionalBin) / double(size);
                const double imageOmega = M_PI - omega;

                // Re-project at the interpolated frequency.  This avoids a
                // 122 Hz FFT-bin quantization error; a few tens of hertz are
                // enough to bias the very deep image null we are seeking.
                std::complex<double> desired(0.0, 0.0);
                std::complex<double> desiredPair(0.0, 0.0);
                std::complex<double> image(0.0, 0.0);
                std::complex<double> imagePair(0.0, 0.0);
                std::complex<double> desiredRotation(1.0, 0.0);
                std::complex<double> desiredPairRotation(1.0, 0.0);
                std::complex<double> imageRotation(1.0, 0.0);
                std::complex<double> imagePairRotation(1.0, 0.0);
                const auto desiredStep = std::polar(1.0, -omega);
                const auto desiredPairStep = std::polar(1.0, -(omega - M_PI));
                const auto imageStep = std::polar(1.0, -imageOmega);
                const auto imagePairStep = std::polar(
                    1.0, -(imageOmega - M_PI));
                sampleIndex = 0;
                for (std::size_t frameIndex = REAL_TIMING_WARMUP_FRAMES;
                     frameIndex < state->timingCalibration.size();
                     ++frameIndex) {
                    const auto &frame = state->timingCalibration[frameIndex];
                    for (std::size_t i = 0; i < REAL8_SAMPLES; ++i) {
                        const double mean = (i & 1u) ? oddMean : evenMean;
                        const double window = 0.5 - 0.5 * std::cos(
                            2.0 * M_PI * double(sampleIndex) /
                            double(calibrationSamples - 1));
                        const double value = (frame[i] - mean) * window;
                        desired += value * desiredRotation;
                        desiredPair += value * desiredPairRotation;
                        image += value * imageRotation;
                        imagePair += value * imagePairRotation;
                        desiredRotation *= desiredStep;
                        desiredPairRotation *= desiredPairStep;
                        imageRotation *= imageStep;
                        imagePairRotation *= imagePairStep;
                        ++sampleIndex;
                    }
                }

                auto coefficient = [](std::size_t tap, double scale) {
                    double value = REAL_TIMING_PHASE[tap];
                    if (tap == REAL_TIMING_TAPS / 2) {
                        value = 1.0 + scale * (value - 1.0);
                    } else {
                        value *= scale;
                    }
                    return value;
                };
                auto response = [&](double frequency, bool forward,
                                    double scale) {
                    std::complex<double> value(0.0, 0.0);
                    for (std::size_t tap = 0; tap < REAL_TIMING_TAPS; ++tap) {
                        const std::size_t index = forward
                            ? tap : REAL_TIMING_TAPS - 1 - tap;
                        const int offset = int(tap) -
                                           int(REAL_TIMING_TAPS / 2);
                        value += coefficient(index, scale) *
                                 std::polar(1.0, frequency * double(offset));
                    }
                    return value;
                };
                auto periodicComponents = [&](double frequency,
                                              int candidateSign,
                                              double scale) {
                    const bool evenForward = candidateSign > 0;
                    const auto even = response(frequency, evenForward, scale);
                    const auto odd = response(frequency, !evenForward, scale);
                    const auto directResponse = 0.5 * (even + odd);
                    const double pairedFrequency = frequency - M_PI;
                    const auto pairedEven = response(
                        pairedFrequency, evenForward, scale);
                    const auto pairedOdd = response(
                        pairedFrequency, !evenForward, scale);
                    const auto pairedResponse =
                        0.5 * (pairedEven - pairedOdd);
                    return std::make_pair(directResponse, pairedResponse);
                };
                auto periodicOutput = [](std::complex<double> direct,
                                         std::complex<double> paired,
                                         const auto &components,
                                         double oddGain) {
                    const double common = 0.5 * (1.0 + oddGain);
                    const double alternating = 0.5 * (1.0 - oddGain);
                    const auto gainedDirect =
                        common * direct + alternating * paired;
                    const auto gainedPair =
                        common * paired + alternating * direct;
                    return components.first * gainedDirect +
                           components.second * gainedPair;
                };

                double bestRatio = 0.0;
                int bestSign = sign;
                double bestScale = 1.0;
                double bestGain = sign > 0 ? 0.95 : 1.0 / 0.95;
                for (int candidateSign : {-1, 1}) {
                    for (int skewStep = 0; skewStep <= 560; ++skewStep) {
                        const double scale = double(skewStep) / 400.0;
                        const auto desiredComponents = periodicComponents(
                            omega, candidateSign, scale);
                        const auto imageComponents = periodicComponents(
                            imageOmega, candidateSign, scale);
                        for (int gainStep = 680; gainStep <= 920; ++gainStep) {
                            const double gain = double(gainStep) / 800.0;
                            const auto correctedDesired = periodicOutput(
                                desired, desiredPair, desiredComponents, gain);
                            const auto correctedImage = periodicOutput(
                                image, imagePair, imageComponents, gain);
                            const double ratio = std::norm(correctedDesired) /
                                std::max(std::norm(correctedImage), 1e-30);
                            if (ratio > bestRatio) {
                                bestRatio = ratio;
                                bestSign = candidateSign;
                                bestScale = scale;
                                bestGain = gain;
                            }
                        }
                    }
                }
                sign = bestSign;
                state->timingCoefficientScale = float(bestScale);
                state->timingOddGain = float(bestGain);
                state->timingSkewPpm = static_cast<int>(std::lround(
                    bestSign * bestScale * 212500.0));
                state->timingOddGainPpm = static_cast<int>(std::lround(
                    bestGain * 1e6));
            }
        }
        return sign;
    }
    static float correctRealSample(StreamState *state, float sample)
    {
        const bool targetEven = (state->mixerPhase & 1u) == 0u;
        sample = (sample - state->timingOffset[targetEven ? 0u : 1u]) * 4.0f;
        if (!targetEven) sample *= state->timingOddGain;
        state->timingHistoryPosition =
            (state->timingHistoryPosition + 1) % REAL_TIMING_TAPS;
        const std::size_t pos = state->timingHistoryPosition;
        state->timingHistory[pos] =
            state->timingHistory[pos + REAL_TIMING_TAPS] = sample;
        const float *current = state->timingHistory.data() +
                               pos + REAL_TIMING_TAPS;
        const bool forward =
            targetEven == (state->timingSkewSign.load() > 0);
        float corrected = 0.0f;
        for (std::size_t tap = 0; tap < REAL_TIMING_TAPS; ++tap) {
            const std::size_t coefficient =
                forward ? tap : REAL_TIMING_TAPS - 1 - tap;
            float value = REAL_TIMING_PHASE[coefficient];
            if (coefficient == REAL_TIMING_TAPS / 2) {
                value = 1.0f + state->timingCoefficientScale * (value - 1.0f);
            } else {
                value *= state->timingCoefficientScale;
            }
            corrected += value *
                         current[-static_cast<std::ptrdiff_t>(
                             REAL_TIMING_TAPS - 1 - tap)];
        }
        return corrected;
    }
    static void processRealFrame(StreamState *state, const uint8_t *samples,
                                 SampleBlock &block)
    {
        block.samples = REAL8_SAMPLES / 2;
        auto push = [state](float real, float imag) {
            state->historyPosition = (state->historyPosition + 1) % 95;
            const std::size_t pos = state->historyPosition;
            state->realHistory[pos] = state->realHistory[pos + 95] = real;
            state->imagHistory[pos] = state->imagHistory[pos + 95] = imag;
        };
        for (std::size_t output = 0; output < REAL8_SAMPLES / 2; ++output) {
            float x = correctRealSample(
                state, float(int8_t(samples[2 * output])));
            // e^(-j*pi*n/2): 1, -j, -1, +j. Frame length is a
            // multiple of four, so phase continuity follows frame continuity.
            if (state->mixerPhase == 0) push(x, 0.0f);
            else push(-x, 0.0f); // phase 2
            state->mixerPhase = (state->mixerPhase + 1) & 3u;

            const float *real = state->realHistory.data() +
                                state->historyPosition + 95;
            const float *imag = state->imagHistory.data() +
                                state->historyPosition + 95;
            float acc = 0.0f;
            for (std::size_t tap = 0; tap < REAL_HALF_BAND.size(); ++tap) {
                const std::size_t index = tap * 2;
                acc += REAL_HALF_BAND[tap] *
                       (real[-static_cast<std::ptrdiff_t>(index)] +
                        real[-static_cast<std::ptrdiff_t>(94 - index)]);
            }
            const long i = std::lround(2.0f * acc);
            const long q = std::lround(imag[-47]);
            block.iq[2 * output] = static_cast<int16_t>(
                std::clamp(i, -512l, 511l));
            block.iq[2 * output + 1] = static_cast<int16_t>(
                std::clamp(q, -512l, 511l));

            x = correctRealSample(
                state, float(int8_t(samples[2 * output + 1])));
            if (state->mixerPhase == 1) push(0.0f, -x);
            else push(0.0f, x); // phase 3
            state->mixerPhase = (state->mixerPhase + 1) & 3u;
        }
    }
    static void finishFrame(StreamState *state, PendingFrame &&pending)
    {
        const bool full = pending.data.size() == IQ_FRAME_BYTES && std::memcmp(pending.data.data(), "IQC1", 4) == 0;
        const bool compressed = pending.data.size() == IQ8_FRAME_BYTES && std::memcmp(pending.data.data(), "IQC8", 4) == 0;
        const bool real = pending.data.size() == REAL8_FRAME_BYTES && std::memcmp(pending.data.data(), "IQR8", 4) == 0;
        if (!validFrame(pending) || (!full && !compressed && !real)) {
            state->invalidDatagrams++;
            return;
        }
        const uint32_t source = le32(pending.data.data() + 8);
        SampleBlock block;
        const uint8_t *sampleData = pending.data.data() + IQ_HEADER_BYTES;
        if (compressed) {
            // Interleaved int8 I,Q pairs holding the top 8 of 10 sample
            // bits; scale to the canonical 10-bit range.
            for (std::size_t i = 0; i < IQ_SAMPLES * 2; ++i) {
                block.iq[i] = int16_t(int8_t(sampleData[i])) * 4;
            }
        } else if (full) {
            for (std::size_t i = 0; i < IQ_SAMPLES; ++i) {
                const uint32_t word = le32(sampleData + i * 4);
                // The dump word stores Q in bits 9:0 and I in bits 19:10;
                // bits 27:20 carry gain and bits 31:28 carry AGC state.
                // Soapy complex formats are interleaved I,Q.
                block.iq[i * 2] = signExtend10(word >> 10);
                block.iq[i * 2 + 1] = signExtend10(word);
            }
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
                resetRealDsp(state);
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
        auto enqueue = [state](SampleBlock &&ready) {
            if (state->queue.size() >= MAX_QUEUE_BLOCKS) {
                state->queue.pop_front();
                state->queueDrops++;
                state->overflowPending = true;
            }
            state->queue.emplace_back(std::move(ready));
        };
        if (real && state->timingSkewSign.load() == 0) {
            std::array<int8_t, REAL8_SAMPLES> raw{};
            std::memcpy(raw.data(), sampleData, raw.size());
            state->timingCalibration.emplace_back(std::move(raw));
            if (state->timingCalibration.size() ==
                REAL_TIMING_CALIBRATION_FRAMES) {
                state->timingSkewSign = detectTimingSkew(state);
                for (const auto &calibration : state->timingCalibration) {
                    SampleBlock corrected;
                    processRealFrame(
                        state,
                        reinterpret_cast<const uint8_t *>(calibration.data()),
                        corrected);
                    enqueue(std::move(corrected));
                }
                state->timingCalibration.clear();
            }
        } else {
            if (real) processRealFrame(state, sampleData, block);
            enqueue(std::move(block));
        }
        state->completedFrames++;
        state->condition.notify_all();
    }
    static void handleDatagram(StreamState *state, const uint8_t *data, std::size_t bytes)
    {
        state->datagrams++;
        UdpHeader header;
        if (!parseHeader(data, bytes, header)) { state->invalidDatagrams++; return; }
        if (std::memcmp(header.frameMagic.data(), "IQC1", 4) != 0 &&
            std::memcmp(header.frameMagic.data(), "IQC8", 4) != 0 &&
            std::memcmp(header.frameMagic.data(), "IQR8", 4) != 0) return;
        if (!state->haveEpoch || state->epoch != header.epoch) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->epoch = header.epoch;
            state->haveEpoch = true;
            state->pending.clear();
            state->queue.clear();
            state->haveExpectedSource = false;
            state->haveFirmwareDropped = false;
            state->haveMinimumFrameSequence = false;
            resetRealDsp(state);
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
                resetRealDsp(state);
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
