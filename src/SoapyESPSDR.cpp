#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Version.hpp>

#include "TimestampUnwrapper.hpp"

#include <curl/curl.h>
#include <json/json.h>
#include <libusb-1.0/libusb.h>
#include <zlib.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <complex>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

class TimedTxError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

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
constexpr double RX_BASE_SAMPLE_RATE = 16e6;
constexpr std::size_t MAX_FRAME_BYTES = 64 * 1024;
// Absorb host-side scheduling stalls without discarding RF frames. At 16 MS/s
// this provides about 260 ms of elasticity.
constexpr std::size_t MAX_QUEUE_BLOCKS = 4096;
constexpr uint32_t UDP_VERSION = 1;
constexpr uint32_t IQ_FLAG_TIMESTAMP_US32 = 1u << 31;
constexpr uint32_t IQ_FLAG_SOFTWARE_AGC_ACTIVE = 1u << 30;
constexpr unsigned IQ_AGC_ROBUST_PEAK_SHIFT = 20;
constexpr uint32_t IQ_AGC_ROBUST_PEAK_MASK = 0x3ffu;
constexpr unsigned IQ_AGC_GAIN_CHANGES_SHIFT = 4;
constexpr uint32_t IQ_AGC_GAIN_CHANGES_MASK = 0xffffu;
constexpr std::size_t TX_UDP_HEADER_BYTES = 36;
constexpr std::size_t TX_UDP_ACK_BYTES = 40;
constexpr std::size_t TX_UDP_WORDS_PER_DATAGRAM = 350;
constexpr std::size_t TX_UDP_ACK_WINDOW_DATAGRAMS = 256;
constexpr std::size_t TX_UDP_PACING_BURST_DATAGRAMS = 16;
constexpr std::size_t TX_REPLAY_SEGMENT_SAMPLES = 16383;
constexpr std::size_t TX_REPLAY_MAX_SAMPLES = TX_REPLAY_SEGMENT_SAMPLES * 64;
constexpr std::size_t TX_STREAM_BATCH_SAMPLES = 524288;
constexpr uint16_t TX_UDP_FLAG_AUTOSTART = 1u << 8;
constexpr unsigned TX_UDP_RATE_CODE_SHIFT = 9;
constexpr uint16_t TX_UDP_FLAG_MORE = 1u << 13;
constexpr uint16_t TX_UDP_FLAG_CONTINUE = 1u << 14;
constexpr unsigned STREAM_SELECTION_MAX = 1'000'000;
constexpr uint32_t S31_ADC_SOURCE_MUX_MASK = 0x0000000fu;
constexpr std::size_t REAL_TIMING_TAPS = 25;

/* The six-bit RFTX2 PBUS value is a segmented hardware code, not dB: output
 * rises inside each eight-code bank and falls at the bank boundary.  These
 * points are a conservative monotonic subset measured over the air at
 * 2.38 GHz.  Values are relative power gain from code 1; they intentionally
 * stop below the vendor calibration code (23), which can brown out a
 * marginally powered development board. */
struct TxGainPoint {
    double db;
    unsigned code;
};
constexpr std::array<TxGainPoint, 15> TX_GAIN_POINTS{{
    {0.00, 1}, {3.44, 2}, {5.99, 3}, {7.92, 4}, {9.78, 17},
    {10.85, 6}, {11.97, 7}, {13.14, 18}, {13.98, 12},
    {15.53, 19}, {16.70, 14}, {17.23, 20}, {17.73, 15},
    {18.62, 21}, {19.57, 22},
}};

unsigned txGainCode(const double requestedDb)
{
    const auto point = std::min_element(
        TX_GAIN_POINTS.begin(), TX_GAIN_POINTS.end(),
        [requestedDb](const TxGainPoint &a, const TxGainPoint &b) {
            return std::abs(a.db - requestedDb) <
                   std::abs(b.db - requestedDb);
        });
    return point->code;
}

double txGainDb(const unsigned code)
{
    const auto point = std::find_if(
        TX_GAIN_POINTS.begin(), TX_GAIN_POINTS.end(),
        [code](const TxGainPoint &candidate) {
            return candidate.code == code;
        });
    return point == TX_GAIN_POINTS.end() ? NAN : point->db;
}
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
constexpr uint8_t USB_EP_TX_OUT = 0x02;
constexpr std::size_t USB_CTRL_HEADER_BYTES = 16;
constexpr std::size_t USB_CTRL_MAX_PAYLOAD = 2048;
constexpr unsigned USB_CTRL_TIMEOUT_MS = 3000;
constexpr int USB_STREAM_TRANSFERS = 8;
constexpr std::size_t USB_STREAM_TRANSFER_BYTES = 256 * 1024;
constexpr std::size_t USB_TX_TRANSFER_BYTES = 4 * 1024;

enum UsbControlOpcode : uint32_t {
    USB_OP_GET_STATUS = 1,
    USB_OP_GET_CONFIG = 2,
    USB_OP_PUT_CONFIG = 3,
    USB_OP_STREAM_START = 4,
    USB_OP_STREAM_STOP = 5,
    USB_OP_TX_ARM = 6,
    USB_OP_TX_COMMIT = 7,
    USB_OP_TX_ABORT = 8,
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

void putLe16(uint8_t *p, uint16_t value)
{
    p[0] = value & 0xff;
    p[1] = (value >> 8) & 0xff;
}

void putLe32(uint8_t *p, uint32_t value)
{
    p[0] = value & 0xff;
    p[1] = (value >> 8) & 0xff;
    p[2] = (value >> 16) & 0xff;
    p[3] = (value >> 24) & 0xff;
}

void putLe64(uint8_t *p, uint64_t value)
{
    putLe32(p, static_cast<uint32_t>(value));
    putLe32(p + 4, static_cast<uint32_t>(value >> 32));
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
    HttpClient(std::string host, unsigned port, std::string interface = {}):
        _host(std::move(host)), _port(port), _interface(std::move(interface))
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
        if (!_interface.empty())
            curl_easy_setopt(curl.get(), CURLOPT_INTERFACE,
                             _interface.c_str());
        CURLcode result = CURLE_OK;
        for (unsigned attempt = 0; attempt < 3; ++attempt) {
            response.clear();
            result = curl_easy_perform(curl.get());
            if (result == CURLE_OK) break;
            const bool transient =
                result == CURLE_COULDNT_RESOLVE_HOST ||
                result == CURLE_COULDNT_CONNECT ||
                result == CURLE_OPERATION_TIMEDOUT ||
                result == CURLE_SEND_ERROR ||
                result == CURLE_RECV_ERROR ||
                result == CURLE_GOT_NOTHING;
            if (!transient || attempt + 1 == 3) break;
            /* The ESP HTTP server can briefly have all sockets retiring after
             * a rapid RX/TX ownership sequence. A bounded retry makes normal
             * Soapy control robust without masking protocol or HTTP errors. */
            std::this_thread::sleep_for(
                std::chrono::milliseconds(25u * (attempt + 1u)));
        }
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
    std::string _interface;
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
    HttpControl(std::string host, unsigned port, std::string interface):
        _http(std::move(host), port, std::move(interface)) {}
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

void bindSocketToInterface(int fd, const std::string &interface)
{
    if (interface.empty()) return;
    if (setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, interface.c_str(),
                   interface.size() + 1) != 0)
        throw std::runtime_error("cannot bind socket to interface " +
                                 interface + ": " + std::strerror(errno));
}

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

    Json::Value rawRequest(uint32_t opcode, const uint8_t *payload,
                           std::size_t payloadBytes)
    {
        return requestBytes(opcode, payload, payloadBytes);
    }

private:
    Json::Value request(uint32_t opcode, const Json::Value *body)
    {
        const std::string payload = body != nullptr ? jsonString(*body) : std::string();
        return requestBytes(opcode,
                            reinterpret_cast<const uint8_t *>(payload.data()),
                            payload.size());
    }

    Json::Value requestBytes(uint32_t opcode, const uint8_t *payload,
                             std::size_t payloadBytes)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (payloadBytes > USB_CTRL_MAX_PAYLOAD)
            throw std::runtime_error("USB control payload too large");
        if (payloadBytes != 0 && payload == nullptr)
            throw std::runtime_error("USB control payload is null");
        // Keep the request off exact packet-size multiples so the transfer
        // always terminates with a short packet. Padding is outside the
        // logical payload so binary control messages remain exact.
        const std::size_t padding =
            (USB_CTRL_HEADER_BYTES + payloadBytes) % 512 == 0 ? 1 : 0;
        const uint32_t sequence = ++_sequence;
        std::vector<uint8_t> out(USB_CTRL_HEADER_BYTES + payloadBytes + padding);
        std::memcpy(out.data(), "IQRQ", 4);
        putLe32(out.data() + 4, sequence);
        putLe32(out.data() + 8, opcode);
        putLe32(out.data() + 12, static_cast<uint32_t>(payloadBytes));
        if (payloadBytes != 0)
            std::memcpy(out.data() + USB_CTRL_HEADER_BYTES, payload,
                        payloadBytes);
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
        const uint32_t responsePayloadBytes = le32(in.data() + 12);
        if (USB_CTRL_HEADER_BYTES + responsePayloadBytes >
            static_cast<std::size_t>(transferred)) {
            throw std::runtime_error("USB control response truncated");
        }
        const std::string text(
            reinterpret_cast<char *>(in.data()) + USB_CTRL_HEADER_BYTES,
            responsePayloadBytes);
        if (errorStatus != 0) throw std::runtime_error("ESP-SDR USB control error: " + text);
        return text.empty() ? Json::Value(Json::objectValue) : parseJson(text);
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
    uint64_t timeNs = 0;
    uint32_t sampleRateHz = 0;
    bool hasTime = false;
};

struct RxDatagram {
    std::array<uint8_t, 2048> data{};
    uint16_t bytes = 0;
};

struct TxBurstStatus {
    int result = 0;
    bool hasTime = false;
    long long timeNs = 0;
};

constexpr uint32_t RX_DATAGRAM_RING_SIZE = 8192;

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
    int direction = SOAPY_SDR_RX;
    std::string format;
    int socketFd = -1;
    uint16_t port = 0;
    std::shared_ptr<UsbContext> usb; // non-null when streaming over USB
    std::vector<uint8_t> usbParseBuffer;
    std::atomic<int> usbActiveTransfers{0};
    std::atomic<bool> active{false};
    std::atomic<bool> stop{false};
    std::thread worker;
    std::thread decoderWorker;
    std::vector<RxDatagram> rxDatagramRing;
    std::atomic<uint32_t> rxDatagramHead{0};
    std::atomic<uint32_t> rxDatagramTail{0};
    std::mutex rxWakeMutex;
    std::condition_variable rxWakeCondition;
    std::mutex mutex;
    std::condition_variable condition;
    std::deque<SampleBlock> queue;
    // Consecutive TX writes without a boundary flag are fragments of one
    // finite hardware replay batch. This avoids turning the small buffers
    // used by ordinary Soapy applications into separate RF/config bursts.
    std::mutex txWriteMutex;
    std::vector<uint32_t> txPendingWords;
    std::atomic<uint32_t> txBufferedSamples{0};
    bool txPendingHasTime = false;
    long long txPendingStartTimeNs = 0;
    long long txPendingActualTimeNs = 0;
    bool txChainHasTime = false;
    uint64_t txChainSubmittedSamples = 0;
    uint32_t txChainSubmittedSegments = 0;
    bool txHaveLastContinuationCommit = false;
    std::chrono::steady_clock::time_point txLastContinuationCommit;
    // writeStream() is synchronous, so a successful finite burst has already
    // reached RF when it is reported through readStreamStatus().
    std::deque<TxBurstStatus> txBurstStatuses;
    // The production IQC8 stream is strictly ordered and has two fragments
    // per frame.  Reuse one assembly buffer for that hot path instead of
    // allocating and updating std::map nodes 31,000 times per second.  The
    // map remains as the reorder-capable fallback for other wire formats.
    PendingFrame sequentialPending;
    bool sequentialPendingActive = false;
    std::map<std::pair<uint32_t, uint32_t>, PendingFrame> pending;
    uint32_t epoch = 0;
    bool haveEpoch = false;
    uint32_t expectedDatagramSequence = 0;
    uint32_t lastDatagramSequence = 0;
    bool haveDatagramSequence = false;
    std::set<uint32_t> missingDatagramSequences;
    uint32_t expectedSource = 0;
    bool haveExpectedSource = false;
    espsdr::TimestampUnwrapper timestampUnwrapper;
    uint32_t minimumFrameSequence = 0;
    bool haveMinimumFrameSequence = false;
    uint32_t lastFirmwareDropped = 0;
    bool haveFirmwareDropped = false;
    bool overflowPending = false;
    std::atomic<uint64_t> datagrams{0};
    std::atomic<uint64_t> invalidDatagrams{0};
    std::atomic<uint64_t> duplicateDatagrams{0};
    std::atomic<uint64_t> reorderedDatagrams{0};
    std::atomic<uint64_t> datagramGaps{0};
    std::atomic<uint64_t> lateDatagramsRecovered{0};
    std::atomic<uint64_t> unrecoveredDatagramGaps{0};
    std::atomic<uint64_t> completedFrames{0};
    std::atomic<uint64_t> lostChunks{0};
    std::atomic<uint64_t> firmwareDrops{0};
    std::atomic<uint64_t> queueDrops{0};
    std::atomic<uint64_t> captureRestarts{0};
    // Receiver-control telemetry arrives in every IQ frame. This avoids HTTP
    // status traffic competing with full-rate Ethernet samples.
    std::atomic<bool> haveRxTelemetry{false};
    std::atomic<bool> rxSoftwareAgcActive{false};
    std::atomic<uint32_t> rxGain{0};
    std::atomic<uint32_t> rxAgcRobustPeak{0};
    std::atomic<uint32_t> rxAgcGainChanges{0};
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
    std::vector<uint64_t> timingCalibrationTimeNs;
    std::vector<uint32_t> timingCalibrationRateHz;
};

class EspDevice final : public SoapySDR::Device {
public:
    explicit EspDevice(const SoapySDR::Kwargs &args):
        _host(valueOr(args, "host", "esp-sdr.local")),
        _interface(valueOr(args, "interface", "")),
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
            _control = std::make_unique<HttpControl>(_host, _httpPort,
                                                     _interface);
        }
        _config = _control->get("/api/v1/config");
        if (!_config.isObject()) throw std::runtime_error("ESP-SDR returned an invalid configuration");
        const Json::Value status = _control->get("/api/v1/status");
        _status = status;
        _statusRefreshNs = monotonicNanoseconds();
        if (status.isMember("hardware_time_ns")) {
            _hardwareTimeAnchorNs = status["hardware_time_ns"].asInt64();
            _hardwareTimeHostAnchorNs = _statusRefreshNs;
            _haveHardwareTime = true;
        }
        _txUdpAutostart = status.get("tx_udp_autostart", false).asBool();
        const Json::Value gain = status["manual_rx_gain"];
        if (gain.isObject() && gain["unit"].asString() == "dB") {
            _gainMin = gain.get("minimum", 0.0).asDouble();
            _gainMax = gain.get("maximum", 76.0).asDouble();
            _gainStep = gain.get("step", 1.0).asDouble();
        }
        /* Firmware boots in a deliberately sparse 1/2501 idle-safe mode.
         * That is not an application preference: a normal Soapy device must
         * activate as a continuous receiver unless cycle_total/cycle_stream
         * were explicitly supplied in the device arguments. */
        const auto correction = args.find("frequency_correction_ppm");
        if (correction != args.end()) {
            setFrequencyCorrection(SOAPY_SDR_RX, 0,
                                   parseDouble(correction->second,
                                               "frequency_correction_ppm",
                                               -100.0, 100.0));
        }
    }

    ~EspDevice() override
    {
        for (StreamState *stream : {_rxStream, _txStream}) {
            if (stream == nullptr) continue;
            try { deactivateStream(reinterpret_cast<SoapySDR::Stream *>(stream), 0, 0); } catch (...) {}
            closeStream(reinterpret_cast<SoapySDR::Stream *>(stream));
        }
    }

    std::string getDriverKey() const override { return "espsdr"; }
    std::string getHardwareKey() const override { return "ESP-SDR"; }
    SoapySDR::Kwargs getHardwareInfo() const override
    {
        if (_usb != nullptr) {
            return {{"vendor", "Espressif"}, {"hardware", "ESP32-S31 Function-CoreBoard"},
                    {"usb_serial", _usb->serial},
                    {"transport", "USB control / USB RX / USB TX"}};
        }
        return {{"vendor", "Espressif"}, {"hardware", "ESP32-S31 Function-CoreBoard"},
                {"host", _host}, {"transport", "HTTP control / UDP IQ"}};
    }

    bool hasHardwareTime(const std::string &what = "") const override
    {
        if (!what.empty()) return false;
        std::lock_guard<std::mutex> lock(_statusMutex);
        return _haveHardwareTime;
    }
    long long getHardwareTime(const std::string &what = "") const override
    {
        if (!what.empty())
            throw std::runtime_error("unknown hardware time source: " + what);
        std::lock_guard<std::mutex> lock(_statusMutex);
        if (!_haveHardwareTime)
            throw std::runtime_error("firmware does not provide hardware time");
        return _hardwareTimeAnchorNs +
               (monotonicNanoseconds() - _hardwareTimeHostAnchorNs);
    }
    std::size_t getNumChannels(const int direction) const override
    {
        if (direction == SOAPY_SDR_RX) return 1;
        return direction == SOAPY_SDR_TX ? 1 : 0;
    }
    bool getFullDuplex(const int, const std::size_t) const override { return false; }
    std::vector<std::string> listAntennas(
        const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return {"RF"};
    }
    void setAntenna(const int direction, const std::size_t channel,
                    const std::string &name) override
    {
        checkChannel(direction, channel);
        if (name != "RF")
            throw std::runtime_error("ESP-SDR exposes only the shared RF port");
    }
    std::string getAntenna(const int direction,
                           const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return "RF";
    }
    bool hasGainMode(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return direction == SOAPY_SDR_RX;
    }
    void setGainMode(const int direction, const std::size_t channel, const bool automatic) override
    {
        checkChannel(direction, channel);
        if (direction == SOAPY_SDR_TX) {
            if (automatic) throw std::runtime_error("TX gain is manual");
            return;
        }
        Json::Value patch;
        patch["gain"]["gain_mode"] = automatic ? 0 : 1;
        applyPatch(patch);
    }
    bool getGainMode(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        if (direction == SOAPY_SDR_TX) return false;
        return configUInt("gain", "gain_mode", 1) == 0;
    }
    std::vector<std::string> listGains(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return {direction == SOAPY_SDR_RX ? "RX Gain" : "TX Gain"};
    }
    void setGain(const int direction, const std::size_t channel, const double value) override
    {
        setGain(direction, channel,
                direction == SOAPY_SDR_RX ? "RX Gain" : "TX Gain", value);
    }
    void setGain(const int direction, const std::size_t channel, const std::string &name, const double value) override
    {
        checkChannel(direction, channel);
        const char *expected = direction == SOAPY_SDR_RX ? "RX Gain" : "TX Gain";
        if (name != expected) throw std::runtime_error("unknown gain element: " + name);
        if (!std::isfinite(value))
            throw std::runtime_error("gain must be finite");
        Json::Value patch;
        if (direction == SOAPY_SDR_RX) {
            patch["gain"]["gain_mode"] = 1;
            patch["gain"]["rx_gain"] = static_cast<unsigned>(
                std::clamp(std::round(value), _gainMin, _gainMax));
        } else {
            patch["gain"]["tx_gain"] = txGainCode(std::clamp(
                value, TX_GAIN_POINTS.front().db,
                TX_GAIN_POINTS.back().db));
        }
        applyPatch(patch);
    }
    double getGain(const int direction, const std::size_t channel) const override
    {
        return getGain(direction, channel,
                       direction == SOAPY_SDR_RX ? "RX Gain" : "TX Gain");
    }
    double getGain(const int direction, const std::size_t channel, const std::string &name) const override
    {
        checkChannel(direction, channel);
        const char *expected = direction == SOAPY_SDR_RX ? "RX Gain" : "TX Gain";
        if (name != expected) throw std::runtime_error("unknown gain element: " + name);
        return direction == SOAPY_SDR_RX
                   ? configUInt("gain", "rx_gain", 32)
                   : txGainDb(configUInt("gain", "tx_gain", 4));
    }
    SoapySDR::Range getGainRange(const int direction, const std::size_t channel) const override
    {
        return getGainRange(direction, channel,
                            direction == SOAPY_SDR_RX ? "RX Gain" : "TX Gain");
    }
    SoapySDR::Range getGainRange(const int direction, const std::size_t channel, const std::string &name) const override
    {
        checkChannel(direction, channel);
        const char *expected = direction == SOAPY_SDR_RX ? "RX Gain" : "TX Gain";
        if (name != expected) throw std::runtime_error("unknown gain element: " + name);
        return direction == SOAPY_SDR_RX
                   ? SoapySDR::Range(_gainMin, _gainMax, _gainStep)
                   : SoapySDR::Range(TX_GAIN_POINTS.front().db,
                                     TX_GAIN_POINTS.back().db);
    }

    void setFrequency(const int direction, const std::size_t channel, const double frequency,
                      const SoapySDR::Kwargs &) override
    {
        checkChannel(direction, channel);
        if (frequency < 2.3e9 || frequency > 2.8e9) throw std::runtime_error("frequency must be 2300-2800 MHz");
        Json::Value patch;
        patch["radio"]["rf_freq_hz"] = Json::UInt64(std::llround(frequency / 1000.0) * 1000);
        applyPatch(patch);
    }
    void setFrequency(const int direction, const std::size_t channel,
                      const std::string &name, const double frequency,
                      const SoapySDR::Kwargs &args) override
    {
        if (name != "RF") throw std::runtime_error("unknown frequency element: " + name);
        setFrequency(direction, channel, frequency, args);
    }
    double getFrequency(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return configUInt64("radio", "rf_freq_hz", 2412000000u);
    }
    double getFrequency(const int direction, const std::size_t channel,
                        const std::string &name) const override
    {
        if (name != "RF") throw std::runtime_error("unknown frequency element: " + name);
        return getFrequency(direction, channel);
    }
    std::vector<std::string> listFrequencies(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return {"RF"};
    }
    SoapySDR::RangeList getFrequencyRange(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return {{2.3e9, 2.8e9, 1000}};
    }
    SoapySDR::RangeList getFrequencyRange(
        const int direction, const std::size_t channel,
        const std::string &name) const override
    {
        if (name != "RF") throw std::runtime_error("unknown frequency element: " + name);
        return getFrequencyRange(direction, channel);
    }

    bool hasFrequencyCorrection(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return true;
    }
    void setFrequencyCorrection(const int direction, const std::size_t channel,
                                const double value) override
    {
        checkChannel(direction, channel);
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
        checkChannel(direction, channel);
        return configInt64("radio", "frequency_correction_ppb", 0) / 1000.0;
    }

    void setSampleRate(const int direction, const std::size_t channel, const double rate) override
    {
        checkChannel(direction, channel);
        if (direction == SOAPY_SDR_TX) {
            for (const double supported : txSampleRates()) {
                if (std::abs(rate - supported) < 1000.0) {
                    _txSampleRate = supported;
                    return;
                }
            }
            throw std::runtime_error("unsupported TX sample rate");
        }
        const unsigned divisor = rxRateDivisor(rate);
        Json::Value patch;
        patch["iq_engine"]["adc_decimation"] = divisor;
        applyPatch(patch);
    }
    double getSampleRate(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        if (direction == SOAPY_SDR_TX) return _txSampleRate;
        const auto divisor = static_cast<unsigned>(std::clamp<int64_t>(
            configInt64("iq_engine", "adc_decimation", 1), 1, 10));
        return RX_BASE_SAMPLE_RATE / divisor;
    }
    std::vector<double> listSampleRates(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return direction == SOAPY_SDR_TX ? txSampleRates() : rxSampleRates();
    }
    SoapySDR::RangeList getSampleRateRange(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        SoapySDR::RangeList ranges;
        if (direction == SOAPY_SDR_TX) {
            for (const double rate : txSampleRates()) ranges.emplace_back(rate, rate);
            return ranges;
        }
        for (const double rate : rxSampleRates()) ranges.emplace_back(rate, rate);
        return ranges;
    }

    SoapySDR::ArgInfoList getSettingInfo() const override
    {
        SoapySDR::ArgInfo total;
        total.key = "cycle_total";
        total.value = std::to_string(_cycleTotal.load());
        total.name = "Duty cycle: total chunks";
        total.description = "Total number of chunks in one capture cycle";
        total.units = "chunks";
        total.type = SoapySDR::ArgInfo::INT;
        total.range = SoapySDR::Range(1, STREAM_SELECTION_MAX, 1);
        SoapySDR::ArgInfo streamed = total;
        streamed.key = "cycle_stream";
        streamed.value = std::to_string(_cycleStream.load());
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

        SoapySDR::ArgInfo txGainCodeInfo;
        txGainCodeInfo.key = "tx_gain_code";
        txGainCodeInfo.value = std::to_string(
            configUInt("gain", "tx_gain", 4));
        txGainCodeInfo.name = "Expert raw TX gain code";
        txGainCodeInfo.description =
            "Raw segmented six-bit RFTX2 PBUS code; non-monotonic and "
            "potentially unsafe above the characterized range";
        txGainCodeInfo.type = SoapySDR::ArgInfo::INT;
        txGainCodeInfo.range = SoapySDR::Range(0, 63, 1);

        SoapySDR::ArgInfo filterOverride;
        filterOverride.key = "rx_filter_override";
        filterOverride.value = "0";
        filterOverride.name = "Expert RX filter override";
        filterOverride.description = "Raw firmware path override; normal applications should leave this under driver control";
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

        return {total, streamed, correction, txGainCodeInfo,
                filterOverride, filterMode, filterDcap, adcSource,
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
        } else if (key == "tx_gain_code") {
            Json::Value patch;
            patch["gain"]["tx_gain"] =
                parseUnsigned(value, "tx_gain_code", 0, 63);
            applyPatch(patch);
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
        if (key == "tx_gain_code") return std::to_string(
            configUInt("gain", "tx_gain", 4));
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
        checkChannel(direction, channel);
        if (direction == SOAPY_SDR_TX) {
            const unsigned mhz = static_cast<unsigned>(std::llround(bandwidth / 1e6));
            if (mhz != 20)
                throw std::runtime_error("TX bandwidth is fixed at 20 MHz");
            Json::Value patch;
            patch["bandwidth"]["bw_mhz"] = mhz;
            patch["bandwidth"]["second_chan"] = 0;
            applyPatch(patch);
            return;
        }
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
        patch["rx_filter"]["rx_filter_override"] = 62;
        applyPatch(patch);
    }
    double getBandwidth(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return configUInt(direction == SOAPY_SDR_RX ? "rx_filter" : "bandwidth",
                          direction == SOAPY_SDR_RX ? "filter_bw_mhz" : "bw_mhz",
                          direction == SOAPY_SDR_RX ? 0 : 20) * 1e6;
    }
    std::vector<double> listBandwidths(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        if (direction == SOAPY_SDR_TX) return {20e6};
        std::vector<double> values{0};
        for (unsigned mhz = 13; mhz <= 54; ++mhz) values.push_back(mhz * 1e6);
        return values;
    }
    SoapySDR::RangeList getBandwidthRange(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return direction == SOAPY_SDR_TX ? SoapySDR::RangeList{{20e6, 20e6}}
                                         : SoapySDR::RangeList{{13e6, 54e6, 1e6}};
    }

    std::vector<std::string> getStreamFormats(const int direction, const std::size_t channel) const override
    {
        checkChannel(direction, channel);
        return {SOAPY_SDR_CS16, SOAPY_SDR_CF32, SOAPY_SDR_CS8};
    }
    std::string getNativeStreamFormat(const int direction, const std::size_t channel, double &fullScale) const override
    {
        checkChannel(direction, channel);
        fullScale = direction == SOAPY_SDR_TX ? 32768.0 : 512.0;
        return SOAPY_SDR_CS16;
    }
    SoapySDR::Stream *setupStream(const int direction, const std::string &format,
                                  const std::vector<std::size_t> &channels,
                                  const SoapySDR::Kwargs &) override
    {
        checkChannel(direction, channels.empty() ? 0 : channels.front());
        if (channels.size() > 1) throw std::runtime_error("SoapyESPSDR has one channel per direction");
        if (format != SOAPY_SDR_CS8 && format != SOAPY_SDR_CS16 &&
            format != SOAPY_SDR_CF32) {
            throw std::runtime_error("supported formats are CS16, CF32, and CS8");
        }
        StreamState *&streamSlot = direction == SOAPY_SDR_RX ? _rxStream : _txStream;
        if (streamSlot != nullptr)
            throw std::runtime_error(direction == SOAPY_SDR_RX
                                         ? "one RX stream is already open"
                                         : "one TX stream is already open");
        auto state = std::make_unique<StreamState>();
        state->direction = direction;
        state->format = format;
        if (direction == SOAPY_SDR_TX) {
            streamSlot = state.release();
            return reinterpret_cast<SoapySDR::Stream *>(streamSlot);
        }
        if (_usb != nullptr) {
            state->usb = _usb;
            streamSlot = state.release();
            return reinterpret_cast<SoapySDR::Stream *>(streamSlot);
        }
        state->socketFd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (state->socketFd < 0) throw std::runtime_error("failed to create UDP socket");
        try {
            bindSocketToInterface(state->socketFd, _interface);
        } catch (...) {
            ::close(state->socketFd);
            throw;
        }
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
        streamSlot = state.release();
        return reinterpret_cast<SoapySDR::Stream *>(streamSlot);
    }
    void closeStream(SoapySDR::Stream *stream) override
    {
        auto *state = checkedStream(stream);
        if (state->active) deactivateStream(stream, 0, 0);
        if (state->socketFd >= 0) ::close(state->socketFd);
        StreamState *&streamSlot = state->direction == SOAPY_SDR_RX
                                       ? _rxStream : _txStream;
        streamSlot = nullptr;
        delete state;
    }
    std::size_t getStreamMTU(SoapySDR::Stream *stream) const override
    {
        const auto *state = checkedStream(stream);
        if (state->direction == SOAPY_SDR_TX) return TX_STREAM_BATCH_SAMPLES;
        /* Host reads can aggregate multiple native IQC8 frames, so retain the
         * established 2048-complex-sample block as the public MTU on either
         * transport. */
        return MAX_BLOCK_SAMPLES;
    }
    int activateStream(SoapySDR::Stream *stream, const int flags, const long long, const std::size_t numElems) override
    {
        auto *state = checkedStream(stream);
        if (flags != 0 || numElems != 0) return SOAPY_SDR_NOT_SUPPORTED;
        if (state->active) return 0;
        const StreamState *opposite = state->direction == SOAPY_SDR_RX
                                          ? _txStream : _rxStream;
        if (opposite != nullptr && opposite->active.load()) {
            SoapySDR::logf(SOAPY_SDR_ERROR,
                           "ESP-SDR is half-duplex; deactivate %s before activating %s",
                           state->direction == SOAPY_SDR_RX ? "TX" : "RX",
                           state->direction == SOAPY_SDR_RX ? "RX" : "TX");
            return SOAPY_SDR_STREAM_ERROR;
        }
        if (state->direction == SOAPY_SDR_TX) {
            Json::Value patch;
            patch["tx"]["tx_tone_enable"] = 0;
            applyPatch(patch);
            {
                std::scoped_lock lock(state->mutex, state->txWriteMutex);
                state->txBurstStatuses.clear();
                state->txPendingWords.clear();
                state->txPendingWords.reserve(TX_STREAM_BATCH_SAMPLES);
                state->txBufferedSamples = 0;
                state->txPendingHasTime = false;
                state->txPendingStartTimeNs = 0;
                state->txPendingActualTimeNs = 0;
                state->txChainHasTime = false;
                state->txChainSubmittedSamples = 0;
                state->txChainSubmittedSegments = 0;
                state->txHaveLastContinuationCommit = false;
            }
            state->active = true;
            return 0;
        }
        Json::Value patch;
        patch["stream"]["output_mode"] = 0;
        patch["stream"]["stream_wifi_packets"] = 0;
        patch["trigger"]["trigger_mode"] = 0;
        patch["rx_filter"]["rx_filter_override"] = 62;
        const unsigned total = _cycleTotal.load();
        const unsigned streamed = _cycleStream.load();
        patch["trigger"]["trigger_config"] = intervalTrigger(total, streamed);
        applyPatch(patch);
        state->cycleTotal = total;
        state->cycleStream = streamed;
        const uint64_t timestampReferenceUs = hasHardwareTime()
            ? static_cast<uint64_t>(getHardwareTime() / 1000) : 0u;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->queue.clear();
            state->sequentialPendingActive = false;
            state->pending.clear();
            state->overflowPending = false;
            state->haveEpoch = false;
            state->haveDatagramSequence = false;
            state->missingDatagramSequences.clear();
            state->unrecoveredDatagramGaps = 0;
            state->haveExpectedSource = false;
            state->timestampUnwrapper.reset(timestampReferenceUs);
            state->haveFirmwareDropped = false;
            state->haveMinimumFrameSequence = false;
            state->haveRxTelemetry = false;
            resetRealDsp(state);
        }
        if (state->usb != nullptr) {
            /* Flush any endpoint data left by a crashed or bandwidth-starved
             * previous client before submitting this session's IN transfers.
             * The firmware resets the stream endpoint as part of STOP. */
            _control->post("/api/v1/stream/stop",
                           Json::Value(Json::objectValue));
        }
        state->stop = false;
        state->active = true;
        if (state->usb != nullptr) {
            state->worker = std::thread(&EspDevice::usbReceiveLoop, state);
        } else {
            state->rxDatagramRing.resize(RX_DATAGRAM_RING_SIZE);
            state->rxDatagramHead = 0;
            state->rxDatagramTail = 0;
            state->decoderWorker =
                std::thread(&EspDevice::decodeLoop, state);
            state->worker = std::thread(&EspDevice::receiveLoop, state);
        }
        Json::Value body;
        body["port"] = state->port;
        // The combined S31 image emits native packed IQC8 on both high-speed
        // transports. Request the compact native-IQ framing explicitly.
        body["stream_format"] = 1;
        try {
            _control->post("/api/v1/stream/start", body);
        } catch (...) {
            state->stop = true;
            state->rxWakeCondition.notify_all();
            if (state->worker.joinable()) state->worker.join();
            if (state->decoderWorker.joinable())
                state->decoderWorker.join();
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
        if (state->direction == SOAPY_SDR_TX) {
            int result = 0;
            {
                std::lock_guard<std::mutex> lock(state->txWriteMutex);
                const bool continuationExpired =
                    txContinuationExpired(state);
                if (continuationExpired) {
                    state->txPendingWords.clear();
                    state->txBufferedSamples = 0;
                    state->txPendingHasTime = false;
                    state->txPendingStartTimeNs = 0;
                    state->txPendingActualTimeNs = 0;
                    std::lock_guard<std::mutex> statusLock(state->mutex);
                    state->txBurstStatuses.push_back({
                        SOAPY_SDR_UNDERFLOW, false, 0});
                } else if (!state->txPendingWords.empty()) {
                    try {
                        state->txPendingActualTimeNs = transmitTxWords(
                            state->txPendingWords, 10'000'000,
                            state->txPendingHasTime
                                ? state->txPendingStartTimeNs : 0,
                            false,
                            state->txChainSubmittedSegments != 0);
                    } catch (const std::exception &error) {
                        SoapySDR::logf(
                            SOAPY_SDR_ERROR,
                            "TX deactivation flush failed: %s", error.what());
                        result = SOAPY_SDR_STREAM_ERROR;
                    }
                    state->txPendingWords.clear();
                    state->txBufferedSamples = 0;
                    state->txPendingHasTime = false;
                    state->txPendingStartTimeNs = 0;
                    state->txPendingActualTimeNs = 0;
                }
                state->txChainHasTime = false;
                state->txChainSubmittedSamples = 0;
                state->txChainSubmittedSegments = 0;
                state->txHaveLastContinuationCommit = false;
            }
            Json::Value patch;
            patch["tx"]["tx_tone_enable"] = 0;
            try {
                applyPatch(patch);
            } catch (const std::exception &error) {
                SoapySDR::logf(SOAPY_SDR_ERROR,
                               "TX deactivation failed: %s", error.what());
                result = SOAPY_SDR_STREAM_ERROR;
            }
            state->active = false;
            state->condition.notify_all();
            return result;
        }
        try { _control->post("/api/v1/stream/stop", Json::Value(Json::objectValue)); }
        catch (const std::exception &error) { SoapySDR::logf(SOAPY_SDR_WARNING, "stream stop failed: %s", error.what()); }
        state->stop = true;
        state->rxWakeCondition.notify_all();
        if (state->worker.joinable()) state->worker.join();
        if (state->decoderWorker.joinable()) state->decoderWorker.join();
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
        if (state->direction != SOAPY_SDR_RX) return SOAPY_SDR_NOT_SUPPORTED;
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
        const SampleBlock &firstBlock = state->queue.front();
        if (firstBlock.hasTime && firstBlock.sampleRateHz != 0) {
            flags |= SOAPY_SDR_HAS_TIME;
            timeNs = static_cast<long long>(firstBlock.timeNs) +
                static_cast<long long>(std::llround(
                    static_cast<long double>(firstBlock.offset) * 1.0e9L /
                    firstBlock.sampleRateHz));
        }
        std::size_t produced = 0;
        uint64_t expectedNextTimeNs = 0;
        bool haveExpectedNextTime = false;
        auto timestampIsContiguous = [&](const SampleBlock &block) {
            if (!haveExpectedNextTime || !block.hasTime) return true;
            const uint64_t difference = block.timeNs > expectedNextTimeNs
                ? block.timeNs - expectedNextTimeNs
                : expectedNextTimeNs - block.timeNs;
            return difference <= 2000u;
        };
        auto rememberBlockEnd = [&](const SampleBlock &block) {
            if (!block.hasTime || block.sampleRateHz == 0u) {
                haveExpectedNextTime = false;
                return;
            }
            expectedNextTimeNs = block.timeNs +
                static_cast<uint64_t>(std::llround(
                    static_cast<long double>(block.samples) * 1.0e9L /
                    block.sampleRateHz));
            haveExpectedNextTime = true;
        };
        if (state->format != SOAPY_SDR_CS16) {
            /* CS8/CF32 conversion touches every component. Copy one native
             * block under the queue lock, then convert after releasing it so
             * the 16 MSa/s decoder can enqueue the next frame concurrently.
             * Holding this mutex across scalar conversion can back-pressure
             * the decoder long enough to overflow the datagram SPSC ring. */
            std::array<int16_t, MAX_BLOCK_SAMPLES * 2> native{};
            lock.unlock();
            while (produced < numElems) {
                lock.lock();
                if (state->queue.empty()) {
                    lock.unlock();
                    break;
                }
                SampleBlock &block = state->queue.front();
                if (!timestampIsContiguous(block)) {
                    lock.unlock();
                    break;
                }
                const std::size_t count = std::min(
                    {numElems - produced, block.samples - block.offset,
                     MAX_BLOCK_SAMPLES});
                std::memcpy(native.data(), block.iq.data() + block.offset * 2,
                            count * 2 * sizeof(int16_t));
                block.offset += count;
                if (block.offset == block.samples) {
                    rememberBlockEnd(block);
                    state->queue.pop_front();
                }
                lock.unlock();
                if (state->format == SOAPY_SDR_CS8) {
                    auto *output = static_cast<int8_t *>(buffers[0]);
                    for (std::size_t i = 0; i < count * 2; ++i)
                        output[produced * 2 + i] = int8_t(native[i] / 4);
                } else {
                    auto *output = static_cast<float *>(buffers[0]);
                    for (std::size_t i = 0; i < count * 2; ++i)
                        output[produced * 2 + i] = native[i] / 512.0f;
                }
                produced += count;
            }
            return static_cast<int>(produced);
        }
        while (produced < numElems && !state->queue.empty()) {
            SampleBlock &block = state->queue.front();
            if (!timestampIsContiguous(block)) break;
            const std::size_t count = std::min(numElems - produced,
                                               block.samples - block.offset);
            auto *output = static_cast<int16_t *>(buffers[0]);
            std::memcpy(output + produced * 2,
                        block.iq.data() + block.offset * 2,
                        count * 2 * sizeof(int16_t));
            produced += count;
            block.offset += count;
            if (block.offset == block.samples) {
                rememberBlockEnd(block);
                state->queue.pop_front();
            }
        }
        return static_cast<int>(produced);
    }

    int writeStream(SoapySDR::Stream *stream, const void *const *buffers,
                    const std::size_t numElems, int &flags,
                    const long long timeNs,
                    const long timeoutUs) override
    {
        auto *state = checkedStream(stream);
        if (state->direction != SOAPY_SDR_TX || !state->active)
            return SOAPY_SDR_STREAM_ERROR;
        if ((flags & ~(SOAPY_SDR_END_BURST | SOAPY_SDR_ONE_PACKET |
                       SOAPY_SDR_HAS_TIME)) != 0)
            return SOAPY_SDR_NOT_SUPPORTED;
        if (numElems != 0 && (buffers == nullptr || buffers[0] == nullptr))
            return SOAPY_SDR_STREAM_ERROR;
        if (numElems == 0 &&
            (flags & (SOAPY_SDR_END_BURST | SOAPY_SDR_ONE_PACKET)) == 0)
            return 0;
        const bool endBurst = (flags & SOAPY_SDR_END_BURST) != 0;
        const bool hasTime = (flags & SOAPY_SDR_HAS_TIME) != 0;
        const bool flushRequested =
            (flags & (SOAPY_SDR_END_BURST | SOAPY_SDR_ONE_PACKET)) != 0;
        std::lock_guard<std::mutex> writeLock(state->txWriteMutex);
        if (hasTime && (timeNs < 0 || !state->txPendingWords.empty() ||
                        timeNs <= getHardwareTime())) {
            return SOAPY_SDR_TIME_ERROR;
        }
        if (!hasTime && (numElems != 0 || flushRequested) &&
            state->txPendingHasTime &&
            state->txPendingStartTimeNs <= getHardwareTime()) {
            /* A fragmented application may miss its deadline before sending
             * END_BURST. Discard the unsent batch so deactivation cannot
             * radiate it later as a surprising late transmission. */
            state->txPendingWords.clear();
            state->txBufferedSamples = 0;
            state->txPendingHasTime = false;
            state->txPendingStartTimeNs = 0;
            return SOAPY_SDR_TIME_ERROR;
        }
        std::size_t count = 0;
        bool completedHasTime = false;
        long long completedActualTimeNs = 0;
        int completedStatus = 0;
        try {
            /* Retain one complete host batch until either more input arrives
             * or the application closes the burst. This makes every earlier
             * commit unambiguously MORE while guaranteeing that END_BURST or
             * deactivate can commit a real final (non-MORE) batch. */
            if (state->txPendingWords.size() == TX_STREAM_BATCH_SAMPLES &&
                numElems != 0) {
                if (txContinuationExpired(state)) {
                    state->txPendingWords.clear();
                    state->txBufferedSamples = 0;
                    state->txPendingHasTime = false;
                    state->txPendingStartTimeNs = 0;
                    state->txPendingActualTimeNs = 0;
                    state->txChainHasTime = false;
                    state->txChainSubmittedSamples = 0;
                    state->txChainSubmittedSegments = 0;
                    state->txHaveLastContinuationCommit = false;
                    {
                        std::lock_guard<std::mutex> lock(state->mutex);
                        state->txBurstStatuses.push_back({
                            SOAPY_SDR_UNDERFLOW, false, 0});
                    }
                    state->condition.notify_all();
                    return SOAPY_SDR_UNDERFLOW;
                }
                const bool batchHasTime = state->txPendingHasTime;
                (void)transmitTxWords(
                    state->txPendingWords, timeoutUs,
                    batchHasTime ? state->txPendingStartTimeNs : 0,
                    true, state->txChainSubmittedSegments != 0);
                state->txChainSubmittedSamples +=
                    state->txPendingWords.size();
                ++state->txChainSubmittedSegments;
                state->txLastContinuationCommit =
                    std::chrono::steady_clock::now();
                state->txHaveLastContinuationCommit = true;
                state->txChainHasTime =
                    state->txChainHasTime || batchHasTime;
                state->txPendingWords.clear();
                state->txBufferedSamples = 0;
                state->txPendingHasTime = false;
                state->txPendingStartTimeNs = 0;
            }
            if (hasTime) {
                state->txPendingHasTime = true;
                state->txPendingStartTimeNs = timeNs;
            }
            const std::size_t available =
                TX_STREAM_BATCH_SAMPLES - state->txPendingWords.size();
            count = std::min(numElems, available);
            for (std::size_t i = 0; i < count; ++i) {
                int32_t iv = 0;
                int32_t qv = 0;
                if (state->format == SOAPY_SDR_CS16) {
                    const auto *samples =
                        static_cast<const int16_t *>(buffers[0]);
                    iv = static_cast<int32_t>(
                        std::lround(samples[i * 2] / 64.0));
                    qv = static_cast<int32_t>(
                        std::lround(samples[i * 2 + 1] / 64.0));
                } else if (state->format == SOAPY_SDR_CS8) {
                    const auto *samples =
                        static_cast<const int8_t *>(buffers[0]);
                    iv = static_cast<int32_t>(samples[i * 2]) * 4;
                    qv = static_cast<int32_t>(samples[i * 2 + 1]) * 4;
                } else {
                    const auto *samples =
                        static_cast<const float *>(buffers[0]);
                    iv = static_cast<int32_t>(
                        std::lround(samples[i * 2] * 511.0f));
                    qv = static_cast<int32_t>(
                        std::lround(samples[i * 2 + 1] * 511.0f));
                }
                iv = std::clamp(iv, -512, 511);
                qv = std::clamp(qv, -512, 511);
                state->txPendingWords.push_back(
                    (static_cast<uint32_t>(iv) & 0x3ffu) |
                    ((static_cast<uint32_t>(qv) & 0x3ffu) << 10));
            }
            if (flushRequested && !state->txPendingWords.empty()) {
                completedHasTime =
                    state->txChainHasTime || state->txPendingHasTime;
                completedActualTimeNs = transmitTxWords(
                    state->txPendingWords, timeoutUs,
                    state->txPendingHasTime
                        ? state->txPendingStartTimeNs : 0,
                    false, state->txChainSubmittedSegments != 0);
                state->txChainSubmittedSamples +=
                    state->txPendingWords.size();
                ++state->txChainSubmittedSegments;
                const Json::Value replay = statusSnapshot()["tx_replay"];
                const bool underflow =
                    replay.get("gap_cycles", 0).asUInt() != 0u ||
                    replay.get("words", 0).asUInt64() !=
                        state->txChainSubmittedSamples ||
                    replay.get("segments", 0).asUInt() !=
                        state->txChainSubmittedSegments;
                if (underflow) completedStatus = SOAPY_SDR_UNDERFLOW;
                state->txPendingWords.clear();
                state->txPendingHasTime = false;
                state->txPendingStartTimeNs = 0;
                state->txChainHasTime = false;
                state->txChainSubmittedSamples = 0;
                state->txChainSubmittedSegments = 0;
                state->txHaveLastContinuationCommit = false;
            }
            state->txBufferedSamples = static_cast<uint32_t>(
                state->txPendingWords.size());
        } catch (const TimedTxError &error) {
            /* Firmware is authoritative because a deadline can pass after
             * host validation while a large waveform is still uploading. */
            state->txPendingWords.clear();
            state->txBufferedSamples = 0;
            state->txPendingHasTime = false;
            state->txPendingStartTimeNs = 0;
            state->txChainHasTime = false;
            state->txChainSubmittedSamples = 0;
            state->txChainSubmittedSegments = 0;
            state->txHaveLastContinuationCommit = false;
            SoapySDR::logf(SOAPY_SDR_WARNING, "TX deadline missed: %s",
                           error.what());
            return SOAPY_SDR_TIME_ERROR;
        } catch (const std::exception &error) {
            state->txBufferedSamples = static_cast<uint32_t>(
                state->txPendingWords.size());
            SoapySDR::logf(SOAPY_SDR_ERROR, "TX write failed: %s", error.what());
            return SOAPY_SDR_STREAM_ERROR;
        }
        if (endBurst) {
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->txBurstStatuses.push_back({
                    completedStatus, completedHasTime,
                    completedActualTimeNs});
            }
            state->condition.notify_all();
        }
        if (count == numElems) flags &= SOAPY_SDR_END_BURST;
        return static_cast<int>(count);
    }

    int readStreamStatus(SoapySDR::Stream *stream, std::size_t &chanMask,
                         int &flags, long long &timeNs,
                         const long timeoutUs) override
    {
        auto *state = checkedStream(stream);
        if (state->direction != SOAPY_SDR_TX)
            return SOAPY_SDR_NOT_SUPPORTED;
        std::unique_lock<std::mutex> lock(state->mutex);
        const auto ready = [&]() {
            return !state->txBurstStatuses.empty() || !state->active;
        };
        if (!state->condition.wait_for(
                lock,
                std::chrono::microseconds(std::max<long>(timeoutUs, 0)),
                ready)) {
            return SOAPY_SDR_TIMEOUT;
        }
        if (state->txBurstStatuses.empty())
            return SOAPY_SDR_TIMEOUT;
        const auto status = state->txBurstStatuses.front();
        state->txBurstStatuses.pop_front();
        chanMask = 1u;
        flags = SOAPY_SDR_END_BURST |
                (status.hasTime ? SOAPY_SDR_HAS_TIME : 0);
        timeNs = status.hasTime ? status.timeNs : 0;
        return status.result;
    }

    std::vector<std::string> listSensors() const override
    {
        return {"datagrams", "invalid_datagrams", "duplicate_datagrams",
                "reordered_datagrams", "datagram_gaps",
                "late_datagrams_recovered", "unrecovered_datagram_gaps",
                "completed_frames", "lost_chunks",
                "firmware_drops", "queue_drops", "capture_restarts",
                "rx_agc_active", "rx_agc_current_gain",
                "rx_agc_robust_peak", "rx_agc_gain_changes",
                "timing_skew_sign", "timing_skew_ppm", "timing_odd_gain_ppm",
                "adc_dump_cfg", "adc_dump_mode", "tx_replay_words",
                "tx_replay_segments", "tx_replay_total_cycles",
                "tx_replay_sample_cycles", "tx_replay_gap_cycles",
                "tx_replay_maximum_gap_cycles",
                "tx_replay_deadline_late_max_cycles",
                "tx_replay_duty_ppm",
                "tx_replay_requested_start_time_ns",
                "tx_replay_actual_start_time_ns",
                "tx_replay_start_error_ns",
                "tx_replay_deadline_missed",
                "tx_buffered_samples", "tx_udp_errors", "usb_tx_errors",
                "usb_tx_uploads"};
    }
    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override
    {
        const auto sensors = listSensors();
        if (std::find(sensors.begin(), sensors.end(), key) == sensors.end()) throw std::runtime_error("unknown sensor: " + key);
        SoapySDR::ArgInfo info;
        info.key = key;
        info.name = key;
        if (key == "tx_replay_deadline_missed" || key == "rx_agc_active") {
            info.value = "false";
            info.type = SoapySDR::ArgInfo::BOOL;
        } else {
            info.value = "0";
            info.type = SoapySDR::ArgInfo::INT;
        }
        if (key == "rx_agc_current_gain") {
            info.name = "AGC current RX gain";
            info.description = "Current calibrated receive-gain-table entry";
            info.units = "dB";
        } else if (key == "rx_agc_robust_peak") {
            info.name = "AGC robust peak";
            info.description = "Most recent robust absolute IQ peak used by software AGC";
            info.units = "counts";
        } else if (key == "rx_agc_gain_changes") {
            info.name = "AGC gain changes";
            info.description = "Gain-table changes since the current AGC session started";
            info.units = "changes";
        } else if (key == "rx_agc_active") {
            info.name = "Software AGC active";
            info.description = "True while firmware software AGC controls receive gain";
        }
        return info;
    }
    std::string readSensor(const std::string &key) const override
    {
        if (key == "tx_buffered_samples") {
            return std::to_string(
                _txStream == nullptr ? 0 : _txStream->txBufferedSamples.load());
        }
        if (key == "tx_udp_errors" || key == "usb_tx_errors" ||
            key == "usb_tx_uploads") {
            const Json::Value status = statusSnapshot();
            return std::to_string(status.get(key, 0).asUInt());
        }
        if (key.rfind("rx_agc_", 0) == 0) {
            const StreamState *rx = _rxStream;
            if (rx != nullptr && rx->haveRxTelemetry.load()) {
                if (key == "rx_agc_active")
                    return rx->rxSoftwareAgcActive.load() ? "true" : "false";
                if (key == "rx_agc_current_gain")
                    return std::to_string(rx->rxGain.load());
                if (key == "rx_agc_robust_peak")
                    return std::to_string(rx->rxAgcRobustPeak.load());
                if (key == "rx_agc_gain_changes")
                    return std::to_string(rx->rxAgcGainChanges.load());
            }
            const Json::Value status = statusSnapshot();
            const Json::Value agc = status["software_agc"];
            if (key == "rx_agc_active")
                return agc.get("active", false).asBool() ? "true" : "false";
            if (key == "rx_agc_current_gain")
                return std::to_string(agc.get("current_gain", 0).asUInt());
            if (key == "rx_agc_robust_peak")
                return std::to_string(agc.get("last_robust_peak", 0).asUInt());
            if (key == "rx_agc_gain_changes")
                return std::to_string(agc.get("gain_changes", 0).asUInt());
        }
        if (key.rfind("tx_replay_", 0) == 0) {
            const Json::Value status = statusSnapshot();
            const Json::Value replay = status["tx_replay"];
            if (key == "tx_replay_words") return std::to_string(replay.get("words", 0).asUInt());
            if (key == "tx_replay_segments") return std::to_string(replay.get("segments", 0).asUInt());
            if (key == "tx_replay_total_cycles") return std::to_string(replay.get("total_cycles", 0).asUInt());
            if (key == "tx_replay_sample_cycles") return std::to_string(replay.get("sample_cycles", 0).asUInt());
            if (key == "tx_replay_gap_cycles") return std::to_string(replay.get("gap_cycles", 0).asUInt());
            if (key == "tx_replay_maximum_gap_cycles") return std::to_string(replay.get("maximum_gap_cycles", 0).asUInt());
            if (key == "tx_replay_deadline_late_max_cycles") return std::to_string(replay.get("deadline_late_max_cycles", 0).asUInt());
            if (key == "tx_replay_requested_start_time_ns") return std::to_string(replay.get("requested_start_time_ns", Json::Int64(0)).asInt64());
            if (key == "tx_replay_actual_start_time_ns") return std::to_string(replay.get("actual_start_time_ns", Json::Int64(0)).asInt64());
            if (key == "tx_replay_start_error_ns") return std::to_string(replay.get("start_error_ns", Json::Int64(0)).asInt64());
            if (key == "tx_replay_deadline_missed") return replay.get("deadline_missed", false).asBool() ? "true" : "false";
            if (key == "tx_replay_duty_ppm") {
                const uint64_t total = replay.get("total_cycles", 0).asUInt64();
                const uint64_t samples = replay.get("sample_cycles", 0).asUInt64();
                return std::to_string(total == 0 ? 0 : samples * 1'000'000 / total);
            }
        }
        const StreamState *state = _rxStream != nullptr ? _rxStream : _txStream;
        if (state == nullptr) return "0";
        if (key == "datagrams") return std::to_string(state->datagrams.load());
        if (key == "invalid_datagrams") return std::to_string(state->invalidDatagrams.load());
        if (key == "duplicate_datagrams") return std::to_string(state->duplicateDatagrams.load());
        if (key == "reordered_datagrams") return std::to_string(state->reorderedDatagrams.load());
        if (key == "datagram_gaps") return std::to_string(state->datagramGaps.load());
        if (key == "late_datagrams_recovered") return std::to_string(state->lateDatagramsRecovered.load());
        if (key == "unrecovered_datagram_gaps") return std::to_string(state->unrecoveredDatagramGaps.load());
        if (key == "completed_frames") return std::to_string(state->completedFrames.load());
        if (key == "lost_chunks") return std::to_string(state->lostChunks.load());
        if (key == "firmware_drops") return std::to_string(state->firmwareDrops.load());
        if (key == "queue_drops") return std::to_string(state->queueDrops.load());
        if (key == "capture_restarts") return std::to_string(state->captureRestarts.load());
        if (key == "timing_skew_sign") return std::to_string(state->timingSkewSign.load());
        if (key == "timing_skew_ppm") return std::to_string(state->timingSkewPpm.load());
        if (key == "timing_odd_gain_ppm") return std::to_string(state->timingOddGainPpm.load());
        if (key == "adc_dump_cfg" || key == "adc_dump_mode") {
            const Json::Value status = statusSnapshot();
            return std::to_string(status[key].asUInt());
        }
        throw std::runtime_error("unknown sensor: " + key);
    }

private:
    void cacheStatus(const Json::Value &fresh) const
    {
        std::lock_guard<std::mutex> lock(_statusMutex);
        _status = fresh;
        _statusRefreshNs = monotonicNanoseconds();
        if (fresh.isMember("hardware_time_ns")) {
            _hardwareTimeAnchorNs = fresh["hardware_time_ns"].asInt64();
            _hardwareTimeHostAnchorNs = _statusRefreshNs;
            _haveHardwareTime = true;
        }
    }

    Json::Value statusSnapshot() const
    {
        const int64_t now = monotonicNanoseconds();
        {
            std::lock_guard<std::mutex> lock(_statusMutex);
            /* Full-rate IQ leaves deliberately little GMAC headroom for TCP.
             * Firmware-backed diagnostics are secondary to a lossless sample
             * stream, so retain the last safe snapshot until RX is stopped. */
            if ((_rxStream != nullptr && _rxStream->active.load()) ||
                (now - _statusRefreshNs) < 100'000'000ll) {
                return _status;
            }
        }
        try {
            Json::Value fresh;
            {
                std::lock_guard<std::mutex> lock(_controlMutex);
                fresh = _control->get("/api/v1/status");
            }
            cacheStatus(fresh);
            return fresh;
        } catch (const std::exception &error) {
            std::lock_guard<std::mutex> lock(_statusMutex);
            if (_status.isObject()) {
                SoapySDR::logf(SOAPY_SDR_DEBUG,
                               "using cached ESP-SDR status: %s",
                               error.what());
                return _status;
            }
            throw;
        }
    }

    bool txContinuationExpired(const StreamState *state) const
    {
        if (!state->txHaveLastContinuationCommit ||
            state->txChainSubmittedSegments == 0)
            return false;
        /* A normal USB batch takes about 65 ms to upload and represents at
         * least 118 ms of RF at the fastest supported TX rate. A producer
         * that has supplied nothing for 200 ms has exhausted the two-batch
         * device cushion; accepting its retained batch would splice a large
         * hole into the IQ timeline. Firmware stops 200 ms after its queued
         * RF data is exhausted, which is at least 318 ms after accepting a
         * full-rate continuation batch. Enforce underflow before that
         * earliest stale-restart boundary and avoid extra control transfers
         * while RF is active. The older prolonged starvation path was also
         * observed to brown out real hardware. */
        return std::chrono::steady_clock::now() -
                   state->txLastContinuationCommit >=
               std::chrono::milliseconds(200);
    }

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
    void checkChannel(const int direction, const std::size_t channel) const
    {
        if (channel != 0 || (direction != SOAPY_SDR_RX && direction != SOAPY_SDR_TX))
            throw std::runtime_error("SoapyESPSDR supports channel 0 only");
    }
    const std::vector<double> &txSampleRates() const
    {
        static const std::vector<double> networkRates{
            4e6, 10e6 / 3.0, 2.5e6, 2e6,
        };
        static const std::vector<double> usbRates{
            40e6 / 9.0, 4e6, 10e6 / 3.0, 2.5e6, 2e6,
        };
        return _usb ? usbRates : networkRates;
    }
    static const std::vector<double> &rxSampleRates()
    {
        static const std::vector<double> rates{
            16e6, 8e6, 16e6 / 3.0, 4e6, 3.2e6,
            16e6 / 6.0, 16e6 / 7.0, 2e6, 16e6 / 9.0, 1.6e6,
        };
        return rates;
    }
    static unsigned rxRateDivisor(const double rate)
    {
        for (unsigned divisor = 1; divisor <= 10; ++divisor) {
            if (std::abs(rate - RX_BASE_SAMPLE_RATE / divisor) < 1000.0)
                return divisor;
        }
        throw std::runtime_error(
            "unsupported RX sample rate; use 16 MSa/s divided by an integer 1..10");
    }
    static unsigned txRateCode(const double rate)
    {
        static const std::array<std::pair<double, unsigned>, 11> rates{{
            {80e6, 0}, {40e6, 1}, {80e6 / 3.0, 2}, {20e6, 3},
            {8e6, 7}, {20e6 / 3.0, 8}, {4e6, 9}, {10e6 / 3.0, 10},
            {40e6 / 9.0, 11}, {2.5e6, 12}, {2e6, 13},
        }};
        for (const auto &entry : rates) {
            if (std::abs(rate - entry.first) < 1000.0) return entry.second;
        }
        throw std::runtime_error("unsupported TX sample rate");
    }
    long long uploadTxWordsUsb(const std::vector<uint32_t> &words,
                               const long timeoutUs,
                               const long long startTimeNs,
                               const bool more,
                               const bool continuation)
    {
        std::lock_guard<std::mutex> controlLock(_controlMutex);
        auto *usbControl = static_cast<UsbControl *>(_control.get());
        const auto timeout = std::chrono::microseconds(
            std::max<long>(timeoutUs, 10'000'000));
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        std::array<uint8_t, 16> arm{};
        putLe32(arm.data(), static_cast<uint32_t>(words.size()));
        uint16_t commitFlags = 0;
        if (_txUdpAutostart) {
            commitFlags = TX_UDP_FLAG_AUTOSTART |
                static_cast<uint16_t>(txRateCode(_txSampleRate)
                                      << TX_UDP_RATE_CODE_SHIFT);
            if (more) commitFlags |= TX_UDP_FLAG_MORE;
            if (continuation) commitFlags |= TX_UDP_FLAG_CONTINUE;
        }
        putLe16(arm.data() + 4, commitFlags);
        putLe64(arm.data() + 8, static_cast<uint64_t>(startTimeNs));

        bool armed = false;
        try {
            usbControl->rawRequest(USB_OP_TX_ARM, arm.data(), arm.size());
            armed = true;
            for (std::size_t offset = 0; offset < words.size();) {
                const std::size_t count = std::min(
                    USB_TX_TRANSFER_BYTES / sizeof(uint32_t),
                    words.size() - offset);
                std::vector<uint8_t> payload(count * sizeof(uint32_t));
                for (std::size_t i = 0; i < count; ++i)
                    putLe32(payload.data() + i * sizeof(uint32_t),
                            words[offset + i]);

                const int64_t remainingMs =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        deadline - std::chrono::steady_clock::now()).count();
                if (remainingMs <= 0)
                    throw std::runtime_error("USB TX upload timed out");
                const unsigned transferTimeout = static_cast<unsigned>(
                    std::min<int64_t>(remainingMs, USB_CTRL_TIMEOUT_MS));
                int transferred = 0;
                const int status = libusb_bulk_transfer(
                    _usb->handle, USB_EP_TX_OUT, payload.data(),
                    static_cast<int>(payload.size()), &transferred,
                    transferTimeout);
                if (status != 0 ||
                    transferred != static_cast<int>(payload.size())) {
                    throw std::runtime_error(
                        std::string("USB TX data write failed: ") +
                        libusb_error_name(status));
                }
                offset += count;
            }
            usbControl->rawRequest(USB_OP_TX_COMMIT, nullptr, 0);
            armed = false;

            if (_txUdpAutostart) {
                if (more) return 0;
                while (std::chrono::steady_clock::now() < deadline) {
                    const Json::Value status =
                        _control->get("/api/v1/status");
                    cacheStatus(status);
                    const Json::Value replay = status["tx_replay"];
                    const long long requested = replay.get(
                        "requested_start_time_ns", Json::Int64(0)).asInt64();
                    if (!status.get("config_applying", false).asBool() &&
                        (startTimeNs == 0 || requested == startTimeNs)) {
                        const long long actual = replay.get(
                            "actual_start_time_ns", Json::Int64(0)).asInt64();
                        if (startTimeNs != 0 &&
                            (actual == 0 || replay.get(
                                "deadline_missed", false).asBool()))
                            throw TimedTxError(
                                "deadline passed during USB waveform staging");
                        return actual;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(25));
                }
                throw std::runtime_error(
                    "timed out waiting for USB TX replay completion");
            }
        } catch (...) {
            if (armed) {
                try {
                    usbControl->rawRequest(USB_OP_TX_ABORT, nullptr, 0);
                } catch (...) {
                }
            }
            throw;
        }
        return 0;
    }
    long long uploadTxWords(const std::vector<uint32_t> &words,
                            const long timeoutUs,
                            const long long startTimeNs,
                            const bool more,
                            const bool continuation)
    {
        if (words.empty() || words.size() > TX_REPLAY_MAX_SAMPLES)
            throw std::runtime_error("TX burst must contain 1.." +
                                     std::to_string(TX_REPLAY_MAX_SAMPLES) +
                                     " samples");
        if (_usb != nullptr) {
            return uploadTxWordsUsb(words, timeoutUs, startTimeNs, more,
                                    continuation);
        }

        std::lock_guard<std::mutex> controlLock(_controlMutex);
        Json::Value armRequest(Json::objectValue);
        armRequest["word_count"] = Json::UInt64(words.size());
        if (startTimeNs != 0)
            armRequest["start_time_ns"] = Json::Int64(startTimeNs);
        const Json::Value arm = _control->post(
            "/api/v1/tx/udp/arm", armRequest);
        const uint32_t token = arm["session_token"].asUInt();
        const unsigned port = arm.get("port", 50001).asUInt();
        const uint32_t beforeCommits = arm.get("commit_count", 0).asUInt();
        if (token == 0 || words.size() > arm.get("max_words", 0).asUInt64())
            throw std::runtime_error("invalid TX UDP arm response");

        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        addrinfo *rawAddresses = nullptr;
        const std::string service = std::to_string(port);
        const int resolveResult = getaddrinfo(_host.c_str(), service.c_str(),
                                              &hints, &rawAddresses);
        if (resolveResult != 0)
            throw std::runtime_error("cannot resolve ESP-SDR host: " +
                                     std::string(gai_strerror(resolveResult)));
        std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> addresses(
            rawAddresses, freeaddrinfo);

        int socketFd = -1;
        for (const addrinfo *address = addresses.get(); address != nullptr;
             address = address->ai_next) {
            socketFd = ::socket(address->ai_family, address->ai_socktype,
                                address->ai_protocol);
            if (socketFd < 0) continue;
            try {
                bindSocketToInterface(socketFd, _interface);
            } catch (...) {
                ::close(socketFd);
                throw;
            }
            if (::connect(socketFd, address->ai_addr, address->ai_addrlen) == 0)
                break;
            ::close(socketFd);
            socketFd = -1;
        }
        if (socketFd < 0) throw std::runtime_error("cannot connect TX UDP socket");
        struct SocketCloser {
            void operator()(int *fd) const { if (fd != nullptr) { ::close(*fd); delete fd; } }
        };
        std::unique_ptr<int, SocketCloser> socketGuard(new int(socketFd));
        int sendBuffer = 1 << 20;
        setsockopt(socketFd, SOL_SOCKET, SO_SNDBUF, &sendBuffer,
                   sizeof(sendBuffer));

        static std::atomic<uint32_t> nextBatch{1};
        const uint32_t batch = nextBatch.fetch_add(1);
        /* A first RX->TX transition includes control-plane serialization and
         * RF/replay reconfiguration in addition to the UDP upload. Common
         * SDR applications pass sub-second stream timeouts intended for FIFO
         * waits; treating those as an end-to-end hardware deadline races the
         * firmware and cancels otherwise valid bursts. Preserve a synchronous
         * END_BURST contract with a conservative minimum, while honoring any
         * longer timeout requested by the caller. */
        const auto timeout = std::chrono::microseconds(
            std::max<long>(timeoutUs, 10'000'000));
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::size_t resumeOffset = 0;

        auto receiveAck = [&](int waitMs, std::size_t &received,
                              bool &committed) -> bool {
            pollfd descriptor{socketFd, POLLIN, 0};
            if (::poll(&descriptor, 1, waitMs) <= 0) return false;
            std::array<uint8_t, TX_UDP_ACK_BYTES> ack{};
            const ssize_t bytes = ::recv(socketFd, ack.data(), ack.size(), 0);
            if (bytes != static_cast<ssize_t>(ack.size()) ||
                std::memcmp(ack.data(), "IQA1", 4) != 0 ||
                le16(ack.data() + 4) != 1 ||
                le16(ack.data() + 6) != TX_UDP_ACK_BYTES ||
                le32(ack.data() + 8) != token ||
                le32(ack.data() + 12) != batch ||
                le32(ack.data() + 36) !=
                    static_cast<uint32_t>(crc32(0, ack.data(), 36))) return false;
            received = le32(ack.data() + 20);
            committed = le32(ack.data() + 24) == words.size() &&
                        le32(ack.data() + 32) > beforeCommits;
            return true;
        };

        auto finishCommitted = [&]() -> long long {
            if (!_txUdpAutostart || more) return 0;
            /* The commit ACK means ownership and the replay request were
             * accepted. Wait until the queued replay has completed so an
             * immediate deactivate cannot overwrite that request. */
            while (std::chrono::steady_clock::now() < deadline) {
                const Json::Value status = _control->get("/api/v1/status");
                cacheStatus(status);
                const Json::Value replay = status["tx_replay"];
                const long long requested = replay.get(
                    "requested_start_time_ns", Json::Int64(0)).asInt64();
                if (!status.get("config_applying", false).asBool() &&
                    (startTimeNs == 0 || requested == startTimeNs)) {
                    const long long actual = replay.get(
                        "actual_start_time_ns", Json::Int64(0)).asInt64();
                    if (startTimeNs != 0 &&
                        (actual == 0 || replay.get(
                            "deadline_missed", false).asBool()))
                        throw TimedTxError(
                            "deadline passed during Ethernet waveform staging");
                    return actual;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
            throw std::runtime_error(
                "timed out waiting for TX replay completion");
        };

        bool recovery = false;
        std::array<uint8_t, TX_UDP_HEADER_BYTES +
                                TX_UDP_WORDS_PER_DATAGRAM * 4> packet{};
        while (resumeOffset < words.size() &&
               std::chrono::steady_clock::now() < deadline) {
            const std::size_t windowStart = resumeOffset;
            const std::size_t windowEnd = std::min(
                words.size(), windowStart +
                    TX_UDP_ACK_WINDOW_DATAGRAMS *
                    TX_UDP_WORDS_PER_DATAGRAM);
            const bool reset = windowStart == 0;
            auto pacingDeadline = std::chrono::steady_clock::now();
            for (std::size_t offset = windowStart; offset < windowEnd;
                 offset += TX_UDP_WORDS_PER_DATAGRAM) {
                const std::size_t count = std::min(
                    TX_UDP_WORDS_PER_DATAGRAM, words.size() - offset);
                const bool final = offset + count == words.size();
                const bool windowFinal = offset + count >= windowEnd;
                std::memcpy(packet.data(), "IQT1", 4);
                putLe16(packet.data() + 4, 1);
                putLe16(packet.data() + 6, TX_UDP_HEADER_BYTES);
                putLe32(packet.data() + 8, token);
                putLe32(packet.data() + 12, batch);
                putLe32(packet.data() + 16,
                        static_cast<uint32_t>(offset / TX_UDP_WORDS_PER_DATAGRAM));
                putLe32(packet.data() + 20, static_cast<uint32_t>(offset));
                putLe16(packet.data() + 24, static_cast<uint16_t>(count));
                uint16_t packetFlags = (reset && offset == 0 ? 1u : 0u) |
                                       (final ? 2u : 0u) |
                                       (windowFinal ? 4u : 0u);
                if (final && _txUdpAutostart) {
                    packetFlags |= TX_UDP_FLAG_AUTOSTART |
                        static_cast<uint16_t>(txRateCode(_txSampleRate)
                                              << TX_UDP_RATE_CODE_SHIFT);
                    if (more) packetFlags |= TX_UDP_FLAG_MORE;
                    if (continuation)
                        packetFlags |= TX_UDP_FLAG_CONTINUE;
                }
                putLe16(packet.data() + 26, packetFlags);
                putLe32(packet.data() + 28, 0);
                putLe32(packet.data() + 32,
                        static_cast<uint32_t>(crc32(0, packet.data(), 32)));
                uint8_t *payload = packet.data() + TX_UDP_HEADER_BYTES;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
                std::memcpy(payload, words.data() + offset, count * 4);
#else
                for (std::size_t i = 0; i < count; ++i)
                    putLe32(payload + i * 4, words[offset + i]);
#endif
                const std::size_t packetBytes =
                    TX_UDP_HEADER_BYTES + count * 4;
                if (::send(socketFd, packet.data(), packetBytes, 0) !=
                    static_cast<ssize_t>(packetBytes))
                    throw std::runtime_error("TX UDP send failed");
                // The S31 raw-Ethernet receive path can sustain the stream rate,
                // but a 50 us burst cadence can overrun its descriptor queue.
                // The direct ring-drain firmware sustains 16-frame flights at
                // a 65 us mean cadence (about 21.5 MB/s payload), leaving the
                // control/allocation margin needed by a 16 MB/s IQ stream.
                // Do not use sleep_until() for this sub-millisecond deadline:
                // scheduler wake-up latency accumulated once per Ethernet frame
                // reduces an otherwise lossless 2 MiB upload to well below the
                // radio sample rate on a normal desktop kernel.
                pacingDeadline += std::chrono::microseconds(recovery ? 75 : 65);
                const std::size_t flightPacket =
                    (offset - windowStart) / TX_UDP_WORDS_PER_DATAGRAM + 1;
                if (flightPacket % TX_UDP_PACING_BURST_DATAGRAMS == 0 ||
                    windowFinal) {
                    while (std::chrono::steady_clock::now() < pacingDeadline) {
                        // A full 2 MiB batch occupies one host core for about
                        // 97 ms. Yielding here reintroduces millisecond-scale
                        // scheduling jitter and descriptor-ring bursts.
                    }
                }
            }

            std::size_t received = windowStart;
            bool committed = false;
            const int remainingMs = static_cast<int>(std::max<int64_t>(
                1, std::chrono::duration_cast<std::chrono::milliseconds>(
                       deadline - std::chrono::steady_clock::now()).count()));
            if (receiveAck(std::min(remainingMs, 25), received, committed)) {
                if (committed) return finishCommitted();
                if (received >= windowEnd) {
                    resumeOffset = windowEnd;
                    recovery = false;
                    continue;
                }
            }
            if (received < windowEnd &&
                received % TX_UDP_WORDS_PER_DATAGRAM == 0) {
                resumeOffset = received;
                recovery = true;
            } else {
                /* A cumulative ACK can itself be lost. Replaying only this
                 * bounded window is cheap; its RESET bit also restores a
                 * deterministic session when packet zero was the loss. */
                resumeOffset = windowStart;
                recovery = true;
            }
        }
        throw std::runtime_error("TX UDP commit timed out at " +
                                 std::to_string(resumeOffset) + "/" +
                                 std::to_string(words.size()) + " samples");
    }
    long long transmitTxWords(const std::vector<uint32_t> &words,
                              const long timeoutUs,
                              const long long startTimeNs,
                              const bool more,
                              const bool continuation)
    {
        if (more && !_txUdpAutostart)
            throw std::runtime_error(
                "continuous TX requires firmware autostart support");
        const long long actualTimeNs =
            uploadTxWords(words, timeoutUs, startTimeNs, more,
                          continuation);
        if (_txUdpAutostart) return actualTimeNs;
        Json::Value start;
        start["tx"]["tx_tone0_step"] =
            (txRateCode(_txSampleRate) << 4) | 3u | (1u << 8);
        start["tx"]["tx_tone_enable"] = 2;
        applyPatch(start);
        Json::Value stop;
        stop["tx"]["tx_tone_enable"] = 0;
        applyPatch(stop);
        return 0;
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
        if (_rxStream != nullptr) {
            _rxStream->cycleTotal = total;
            _rxStream->cycleStream = streamed;
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
        if (state == nullptr || (state != _rxStream && state != _txStream))
            throw std::runtime_error("invalid SoapyESPSDR stream");
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
        StreamState *stream = _rxStream;
        const bool active = stream != nullptr && stream->active.load();
        if (active) {
            stream->suppressContinuityUntilNs = monotonicNanoseconds() + 15'000'000'000ll;
        }
        try {
            _control->put("/api/v1/config", patch);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (std::chrono::steady_clock::now() < deadline) {
                const Json::Value status = _control->get("/api/v1/status");
                cacheStatus(status);
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
        /* Native USB IQ bypasses this real-IF reconstruction path. Keep its
         * correction disabled; the 32 MHz Ethernet real-IF compatibility
         * path retains the measured conservative correction. */
        state->timingCoefficientScale = state->usb != nullptr ? 0.0f : 1.0f;
        state->timingOddGain = 1.0f;
        state->timingSkewPpm = 0;
        state->timingOddGainPpm = 1'000'000;
        state->timingCalibration.clear();
        state->timingCalibration.reserve(REAL_TIMING_CALIBRATION_FRAMES);
        state->timingCalibrationTimeNs.clear();
        state->timingCalibrationTimeNs.reserve(REAL_TIMING_CALIBRATION_FRAMES);
        state->timingCalibrationRateHz.clear();
        state->timingCalibrationRateHz.reserve(REAL_TIMING_CALIBRATION_FRAMES);
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
    static void finishIqFrame(StreamState *state, const uint8_t *frame,
                              std::size_t frameBytes)
    {
        const bool full = frameBytes == IQ_FRAME_BYTES &&
            std::memcmp(frame, "IQC1", 4) == 0;
        const bool compressed = frameBytes == IQ8_FRAME_BYTES &&
            std::memcmp(frame, "IQC8", 4) == 0;
        const bool real = frameBytes == REAL8_FRAME_BYTES &&
            std::memcmp(frame, "IQR8", 4) == 0;
        const uint32_t wireCrc = frameBytes >= 4 ?
            le32(frame + frameBytes - 4) : 0;
        if ((!full && !compressed && !real) ||
            (wireCrc != 0 &&
             wireCrc != static_cast<uint32_t>(
                 crc32(0, frame, frameBytes - 4)))) {
            state->invalidDatagrams++;
            return;
        }
        const uint32_t source = le32(frame + 8);
        const uint32_t frameFlags = le32(frame + 32);
        state->rxGain = le32(frame + 28);
        state->rxSoftwareAgcActive =
            (frameFlags & IQ_FLAG_SOFTWARE_AGC_ACTIVE) != 0u;
        state->rxAgcRobustPeak =
            (frameFlags >> IQ_AGC_ROBUST_PEAK_SHIFT) &
            IQ_AGC_ROBUST_PEAK_MASK;
        state->rxAgcGainChanges =
            (frameFlags >> IQ_AGC_GAIN_CHANGES_SHIFT) &
            IQ_AGC_GAIN_CHANGES_MASK;
        state->haveRxTelemetry = true;
        const uint32_t timestampUs32 = le32(frame + 12);
        const uint32_t sampleRateHz = le32(frame + 20);
        SampleBlock block;
        const uint8_t *sampleData = frame + IQ_HEADER_BYTES;
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
        if ((frameFlags & IQ_FLAG_TIMESTAMP_US32) != 0u &&
            sampleRateHz != 0u) {
            const auto extendedUs =
                state->timestampUnwrapper.unwrap(timestampUs32);
            if (extendedUs) {
                block.timeNs = *extendedUs * 1000u;
                block.sampleRateHz = sampleRateHz;
                block.hasTime = true;
            }
        }
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
            state->timingCalibrationTimeNs.push_back(block.timeNs);
            state->timingCalibrationRateHz.push_back(block.sampleRateHz);
            if (state->timingCalibration.size() ==
                REAL_TIMING_CALIBRATION_FRAMES) {
                state->timingSkewSign = detectTimingSkew(state);
                for (std::size_t index = 0;
                     index < state->timingCalibration.size(); ++index) {
                    const auto &calibration = state->timingCalibration[index];
                    SampleBlock corrected;
                    processRealFrame(
                        state,
                        reinterpret_cast<const uint8_t *>(calibration.data()),
                        corrected);
                    corrected.timeNs = state->timingCalibrationTimeNs[index];
                    corrected.sampleRateHz =
                        state->timingCalibrationRateHz[index];
                    corrected.hasTime = corrected.sampleRateHz != 0u;
                    enqueue(std::move(corrected));
                }
                state->timingCalibration.clear();
                state->timingCalibrationTimeNs.clear();
                state->timingCalibrationRateHz.clear();
            }
        } else {
            if (real) processRealFrame(state, sampleData, block);
            enqueue(std::move(block));
        }
        state->completedFrames++;
        state->condition.notify_all();
    }
    static void finishFrame(StreamState *state, const PendingFrame &pending)
    {
        if (std::memcmp(pending.first.frameMagic.data(), "IQB8", 4) == 0) {
            if (pending.data.empty() ||
                pending.data.size() % IQ8_FRAME_BYTES != 0 ||
                pending.first.frameCrc != 0) {
                state->invalidDatagrams++;
                return;
            }
            for (std::size_t offset = 0; offset < pending.data.size();
                 offset += IQ8_FRAME_BYTES) {
                finishIqFrame(state, pending.data.data() + offset,
                              IQ8_FRAME_BYTES);
            }
            return;
        }
        if (!validFrame(pending)) {
            state->invalidDatagrams++;
            return;
        }
        finishIqFrame(state, pending.data.data(), pending.data.size());
    }
    static void handleDatagram(StreamState *state, const uint8_t *data, std::size_t bytes)
    {
        state->datagrams++;
        UdpHeader header;
        if (!parseHeader(data, bytes, header)) { state->invalidDatagrams++; return; }
        if (std::memcmp(header.frameMagic.data(), "IQC1", 4) != 0 &&
            std::memcmp(header.frameMagic.data(), "IQC8", 4) != 0 &&
            std::memcmp(header.frameMagic.data(), "IQR8", 4) != 0 &&
            std::memcmp(header.frameMagic.data(), "IQB8", 4) != 0) return;
        if (!state->haveEpoch || state->epoch != header.epoch) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->epoch = header.epoch;
            state->haveEpoch = true;
            state->sequentialPendingActive = false;
            state->pending.clear();
            state->queue.clear();
            state->haveExpectedSource = false;
            state->haveFirmwareDropped = false;
            state->haveMinimumFrameSequence = false;
            state->haveDatagramSequence = false;
            state->missingDatagramSequences.clear();
            state->unrecoveredDatagramGaps = 0;
            resetRealDsp(state);
        }
        const uint32_t sequence = header.datagramSequence;
        if (state->haveDatagramSequence) {
            if (sequence == state->lastDatagramSequence) {
                state->duplicateDatagrams++;
                return;
            }
            const int32_t displacement = static_cast<int32_t>(
                sequence - state->expectedDatagramSequence);
            if (displacement < 0) {
                auto missing = state->missingDatagramSequences.find(sequence);
                if (missing != state->missingDatagramSequences.end()) {
                    state->missingDatagramSequences.erase(missing);
                    state->lateDatagramsRecovered++;
                    state->unrecoveredDatagramGaps--;
                    state->reorderedDatagrams++;
                } else {
                    // An old sequence which was not an observed hole is a
                    // replayed datagram. Discard it before it can recreate a
                    // frame that was already delivered.
                    state->duplicateDatagrams++;
                    return;
                }
            } else {
                if (displacement > 0) {
                    state->datagramGaps += static_cast<uint32_t>(displacement);
                    state->unrecoveredDatagramGaps += static_cast<uint32_t>(displacement);
                    // Retain exact sequence identities only for small gaps.
                    // Large gaps are still counted but cannot consume
                    // unbounded host memory during a broken link.
                    if (displacement <= 4096) {
                        for (uint32_t missing = state->expectedDatagramSequence;
                             missing != sequence; ++missing) {
                            state->missingDatagramSequences.insert(missing);
                        }
                        while (state->missingDatagramSequences.size() > 8192)
                            state->missingDatagramSequences.erase(
                                state->missingDatagramSequences.begin());
                    }
                }
                state->expectedDatagramSequence = sequence + 1;
            }
        } else {
            state->expectedDatagramSequence = sequence + 1;
            state->haveDatagramSequence = true;
        }
        state->lastDatagramSequence = sequence;
        bool captureRestart = false;
        const unsigned reorderWindow =
            std::max(8u, state->cycleTotal.load() * 2u);
        /* expectedSource and the restart-generation fields are owned by this
         * receive thread. Do not take the sample-queue mutex for every UDP
         * fragment merely to inspect them: at 20 MSa/s that added roughly
         * 40,000 unnecessary lock operations per second and kept the socket
         * from draining NIC aggregation bursts. Only an actual restart needs
         * the lock because it clears host-visible queued samples. */
        if (state->haveExpectedSource &&
            header.sourceChunk < state->expectedSource &&
            state->expectedSource - header.sourceChunk > reorderWindow) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->queue.clear();
            state->haveExpectedSource = false;
            state->captureRestarts++;
            resetRealDsp(state);
            if (monotonicNanoseconds() >=
                state->suppressContinuityUntilNs.load()) {
                state->overflowPending = true;
            }
            state->minimumFrameSequence = header.frameSequence;
            state->haveMinimumFrameSequence = true;
            captureRestart = true;
        }
        if (captureRestart) {
            state->sequentialPendingActive = false;
            state->pending.clear();
        }
        if (state->haveMinimumFrameSequence &&
            static_cast<int32_t>(header.frameSequence - state->minimumFrameSequence) < 0) return;
        if (state->haveFirmwareDropped && header.firmwareDropped > state->lastFirmwareDropped) {
            state->firmwareDrops += header.firmwareDropped - state->lastFirmwareDropped;
        }
        state->lastFirmwareDropped = header.firmwareDropped;
        state->haveFirmwareDropped = true;
        const auto key = std::make_pair(header.epoch, header.frameSequence);

        // Fast production path: Ethernet IQC8 is emitted as two ordered UDP
        // fragments.  A persistent buffer removes per-frame heap and map
        // work from the socket-drain thread, leaving substantially more
        // scheduling margin for short RTL8153 aggregation bursts.
        if (std::memcmp(header.frameMagic.data(), "IQC8", 4) == 0 &&
            header.frameBytes == IQ8_FRAME_BYTES && header.fragmentCount == 2) {
            PendingFrame &sequential = state->sequentialPending;
            const bool matching = state->sequentialPendingActive &&
                sequential.first.epoch == header.epoch &&
                sequential.first.frameSequence == header.frameSequence;
            if (header.fragmentIndex == 0 &&
                state->pending.find(key) == state->pending.end()) {
                sequential.first = header;
                sequential.data.resize(header.frameBytes);
                std::memcpy(sequential.data.data() + header.fragmentOffset,
                            data + UDP_HEADER_BYTES, header.fragmentBytes);
                sequential.receivedCount = 1;
                state->sequentialPendingActive = true;
                return;
            }
            if (matching && sequential.receivedCount == 1) {
                std::memcpy(sequential.data.data() + header.fragmentOffset,
                            data + UDP_HEADER_BYTES, header.fragmentBytes);
                sequential.receivedCount = 2;
                finishFrame(state, sequential);
                state->sequentialPendingActive = false;
                return;
            }
            // A reordered second fragment is uncommon, but preserve the
            // generic assembler's ability to join it if fragment zero follows.
        }
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
            finishFrame(state, pending);
            state->pending.erase(it);
        }
        while (state->pending.size() > 128) {
            state->pending.erase(state->pending.begin());
            std::lock_guard<std::mutex> lock(state->mutex);
            state->overflowPending = true;
        }
    }
    static void receiveLoop(StreamState *state)
    {
        /* Drain a complete NIC/USB aggregation burst per syscall.  At
         * 16 MSa/s IQC8 produces about 31k UDP datagrams/s; one recv() per
         * datagram needlessly increases scheduler and RTL8153 RX-ring
         * pressure, which can drop packets even while SO_RCVBUF is empty. */
        constexpr unsigned BATCH = 128;
        std::array<std::array<uint8_t, 2048>, BATCH> buffers{};
        std::array<iovec, BATCH> iov{};
        std::array<mmsghdr, BATCH> messages{};
        for (unsigned i = 0; i < BATCH; ++i) {
            iov[i].iov_base = buffers[i].data();
            iov[i].iov_len = buffers[i].size();
            messages[i].msg_hdr.msg_iov = &iov[i];
            messages[i].msg_hdr.msg_iovlen = 1;
        }
        while (!state->stop) {
            const int received = recvmmsg(state->socketFd, messages.data(),
                                          BATCH, 0, nullptr);
            if (received <= 0) continue;
            for (int i = 0; i < received; ++i) {
                const uint32_t head =
                    state->rxDatagramHead.load(std::memory_order_relaxed);
                const uint32_t tail =
                    state->rxDatagramTail.load(std::memory_order_acquire);
                if (head - tail >= RX_DATAGRAM_RING_SIZE) {
                    state->invalidDatagrams++;
                    messages[static_cast<unsigned>(i)].msg_len = 0;
                    continue;
                }
                RxDatagram &slot =
                    state->rxDatagramRing[head % RX_DATAGRAM_RING_SIZE];
                const std::size_t bytes =
                    messages[static_cast<unsigned>(i)].msg_len;
                std::memcpy(slot.data.data(),
                            buffers[static_cast<unsigned>(i)].data(), bytes);
                slot.bytes = static_cast<uint16_t>(bytes);
                state->rxDatagramHead.store(head + 1,
                                            std::memory_order_release);
                messages[static_cast<unsigned>(i)].msg_len = 0;
            }
            state->rxWakeCondition.notify_one();
        }
    }
    static void decodeLoop(StreamState *state)
    {
        while (true) {
            uint32_t tail =
                state->rxDatagramTail.load(std::memory_order_relaxed);
            const uint32_t head =
                state->rxDatagramHead.load(std::memory_order_acquire);
            if (tail != head) {
                const RxDatagram &slot =
                    state->rxDatagramRing[tail % RX_DATAGRAM_RING_SIZE];
                handleDatagram(state, slot.data.data(), slot.bytes);
                state->rxDatagramTail.store(tail + 1,
                                            std::memory_order_release);
                continue;
            }
            if (state->stop.load()) break;
            std::unique_lock<std::mutex> lock(state->rxWakeMutex);
            state->rxWakeCondition.wait_for(
                lock, std::chrono::milliseconds(2), [state]() {
                    return state->stop.load() ||
                        state->rxDatagramTail.load(std::memory_order_relaxed) !=
                        state->rxDatagramHead.load(std::memory_order_acquire);
                });
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
                /* Endpoint reset can leave the tail of one cancelled USB
                 * transfer ahead of the new stream epoch. Discard that
                 * startup prefix while acquiring the first valid header;
                 * once epoch lock exists, retain strict corruption counts. */
                if (state->haveEpoch) state->invalidDatagrams++;
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
    std::string _interface;
    unsigned _httpPort;
    unsigned _requestedUdpPort;
    unsigned _rxBufferBytes;
    std::atomic<unsigned> _cycleTotal;
    std::atomic<unsigned> _cycleStream;
    std::shared_ptr<UsbContext> _usb;
    std::unique_ptr<Control> _control;
    mutable std::mutex _controlMutex;
    mutable std::mutex _configMutex;
    mutable std::mutex _statusMutex;
    Json::Value _config;
    mutable Json::Value _status;
    mutable int64_t _statusRefreshNs = 0;
    mutable int64_t _hardwareTimeAnchorNs = 0;
    mutable int64_t _hardwareTimeHostAnchorNs = 0;
    mutable bool _haveHardwareTime = false;
    double _gainMin = 0.0;
    double _gainMax = 76.0;
    double _gainStep = 1.0;
    double _txSampleRate = 4e6;
    bool _txUdpAutostart = false;
    StreamState *_rxStream = nullptr;
    StreamState *_txStream = nullptr;
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
    const bool wantEthernet = args.find("host") != args.end() ||
        (usbArg != args.end() && usbArg->second == "0");

    if (!wantEthernet) {
        for (const std::string &serial : usbEnumerateSerials()) {
            if (usbSerialArg != args.end() && usbSerialArg->second != serial) continue;
            SoapySDR::Kwargs result = args;
            result["driver"] = "espsdr";
            result["usb"] = "1";
            result["usb_serial"] = serial;
            result["label"] = "ESP-SDR USB (" + serial + ")";
            results.push_back(std::move(result));
        }
    }
    if (wantUsb) return results;

    SoapySDR::Kwargs result = args;
    result["driver"] = "espsdr";
    if (result.count("host") == 0) result["host"] = "esp-sdr.local";
    const unsigned port = result.count("http_port") ? static_cast<unsigned>(std::stoul(result["http_port"])) : 80;
    try {
        HttpClient http(result["host"], port,
                        result.count("interface") ? result["interface"] : "");
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
