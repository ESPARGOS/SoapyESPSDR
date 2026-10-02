#include "Transport.hpp"
#include <arpa/inet.h>
#include <curl/curl.h>
#include <deque>
#include <libusb-1.0/libusb.h>
#include <mutex>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
namespace espsdr {
static std::string option(const SoapySDR::Kwargs &args, const char *key,
                          const char *fallback = "") {
    auto it = args.find(key);
    return it == args.end() ? fallback : it->second;
}
static Json::Value parse(const std::string &text) {
    Json::CharReaderBuilder b;
    std::unique_ptr<Json::CharReader> reader(b.newCharReader());
    Json::Value r;
    std::string err;
    if (!reader->parse(text.data(), text.data() + text.size(), &r, &err) || !r.isObject())
        throw std::runtime_error("Invalid ESP-SDR control response: " + err);
    if (r.isMember("error"))
        throw std::runtime_error(r["error"].asString());
    if (r["protocol"].asUInt() != 2)
        throw std::runtime_error("Install the ESP32-S31 streaming firmware (RX protocol v2)");
    if (!r["dc_correction"].isUInt() || r["dc_correction"].asUInt() > 1)
        throw std::runtime_error("Invalid ESP-SDR DC correction setting");
    return r;
}
static std::string encode(const Json::Value &v) {
    Json::StreamWriterBuilder b;
    b["indentation"] = "";
    return Json::writeString(b, v);
}
class Ethernet final : public Transport {
    CURL *curl = nullptr;
    int sock = -1;
    std::mutex mutex;
    std::string host, iface, url;
    static size_t append(char *data, size_t size, size_t n, void *ptr) {
        auto &s = *static_cast<std::string *>(ptr);
        if (size * n > 16384 - s.size())
            return 0;
        s.append(data, size * n);
        return size * n;
    }

  public:
    explicit Ethernet(const SoapySDR::Kwargs &args)
        : host(option(args, "host", "esp-sdr.local")), iface(option(args, "interface")) {
        static const int initialized = [] {
            if (curl_global_init(CURL_GLOBAL_DEFAULT))
                throw std::runtime_error("curl initialization failed");
            return 1;
        }();
        (void)initialized;
        curl = curl_easy_init();
        if (!curl)
            throw std::runtime_error("curl allocation failed");
        url = "http://" + host + ":" + option(args, "http_port", "80") + "/api/rx";
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 2000L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 5000L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append);
        curl_easy_setopt(curl, CURLOPT_IPRESOLVE, CURL_IPRESOLVE_V4);
        if (!iface.empty())
            curl_easy_setopt(curl, CURLOPT_INTERFACE, iface.c_str());
    }
    ~Ethernet() override {
        if (sock >= 0)
            close(sock);
        if (curl)
            curl_easy_cleanup(curl);
    }
    Json::Value request(Json::Value value) override {
        std::lock_guard<std::mutex> lock(mutex);
        const auto body = encode(value);
        std::string response;
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, long(body.size()));
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        auto code = curl_easy_perform(curl);
        if (code != CURLE_OK)
            throw std::runtime_error(curl_easy_strerror(code));
        return parse(response);
    }
    void prepare(Json::Value &start) override {
        if (sock >= 0) {
            close(sock);
            sock = -1;
        }
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
            throw std::runtime_error("Cannot open UDP socket");
        int bytes = 32 * 1024 * 1024;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &bytes, sizeof(bytes));
        if (!iface.empty() &&
            setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, iface.c_str(), iface.size() + 1))
            throw std::runtime_error("Cannot bind UDP interface");
        sockaddr_in local{};
        local.sin_family = AF_INET;
        if (bind(sock, reinterpret_cast<sockaddr *>(&local), sizeof(local)))
            throw std::runtime_error("Cannot bind UDP socket");
        socklen_t size = sizeof(local);
        getsockname(sock, reinterpret_cast<sockaddr *>(&local), &size);
        start["port"] = ntohs(local.sin_port);
        addrinfo hints{}, *resolved = nullptr;
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        if (getaddrinfo(host.c_str(), nullptr, &hints, &resolved))
            throw std::runtime_error("Cannot resolve ESP-SDR host");
        peer = reinterpret_cast<sockaddr_in *>(resolved->ai_addr)->sin_addr.s_addr;
        freeaddrinfo(resolved);
    }
    int receive(uint8_t *data, size_t capacity) override {
        pollfd p{sock, POLLIN, 0};
        int ready = poll(&p, 1, 100);
        if (ready <= 0)
            return ready;
        sockaddr_in source{};
        socklen_t size = sizeof(source);
        auto n = recvfrom(sock, data, capacity, 0, reinterpret_cast<sockaddr *>(&source), &size);
        if (n < 0)
            throw std::runtime_error("UDP receive failed");
        if (source.sin_addr.s_addr != peer)
            return 0;
        if (size_t(n) != packetBytes)
            throw std::runtime_error("Invalid UDP packet length");
        return int(n);
    }
    std::string name() const override { return "ethernet"; }

  private:
    uint32_t peer = 0;
};
class Usb final : public Transport {
    libusb_context *ctx = nullptr;
    libusb_device_handle *handle = nullptr;
    std::mutex mutex;
    struct Read {
        Usb *owner;
        libusb_transfer *transfer = nullptr;
        std::array<uint8_t, 65536> data;
        bool submitted = false;
    };
    // Queue 2 MiB in the host controller (~50 ms at 20 MSa/s including
    // framing). This tolerates short userspace scheduling stalls without
    // exhausting the device's small internal-RAM ring. Partial batches at
    // epoch boundaries can complete reads early; packet assembly handles them.
    std::array<Read, 32> reads{};
    std::mutex readMutex;
    std::deque<Read *> completed;
    static void LIBUSB_CALL completeRead(libusb_transfer *transfer) {
        auto *read = static_cast<Read *>(transfer->user_data);
        std::lock_guard<std::mutex> lock(read->owner->readMutex);
        read->submitted = false;
        read->owner->completed.push_back(read);
    }

  public:
    explicit Usb(const SoapySDR::Kwargs &args) {
        if (libusb_init(&ctx))
            throw std::runtime_error("libusb initialization failed");
        libusb_device **list = nullptr;
        auto count = libusb_get_device_list(ctx, &list);
        auto wanted = option(args, "usb_serial");
        for (ssize_t i = 0; i < count; i++) {
            libusb_device_descriptor d{};
            libusb_get_device_descriptor(list[i], &d);
            if (d.idVendor != usbVid || d.idProduct != usbPid || d.bcdDevice != usbRevision)
                continue;
            libusb_device_handle *candidate = nullptr;
            if (libusb_open(list[i], &candidate))
                continue;
            unsigned char serial[128]{};
            int n = libusb_get_string_descriptor_ascii(candidate, d.iSerialNumber, serial,
                                                       sizeof(serial));
            if (wanted.empty() || (n > 0 && wanted == reinterpret_cast<char *>(serial))) {
                handle = candidate;
                break;
            }
            libusb_close(candidate);
        }
        libusb_free_device_list(list, 1);
        if (!handle) {
            libusb_exit(ctx);
            ctx = nullptr;
            throw std::runtime_error("No accessible ESP-SDR RX v2 USB device");
        }
        if (libusb_get_device_speed(libusb_get_device(handle)) != LIBUSB_SPEED_HIGH ||
            libusb_claim_interface(handle, 0)) {
            libusb_close(handle);
            libusb_exit(ctx);
            throw std::runtime_error("Cannot claim high-speed ESP-SDR USB interface");
        }
    }
    ~Usb() override {
        finish();
        if (handle) {
            libusb_release_interface(handle, 0);
            libusb_close(handle);
        }
        if (ctx)
            libusb_exit(ctx);
    }
    Json::Value request(Json::Value value) override {
        std::lock_guard<std::mutex> lock(mutex);
        auto body = encode(value);
        int transferred = 0;
        if (body.size() >= 512)
            throw std::runtime_error("Control request too large");
        int rc = libusb_bulk_transfer(handle, 0x01, reinterpret_cast<unsigned char *>(body.data()),
                                      int(body.size()), &transferred, 5000);
        if (rc || transferred != int(body.size()))
            throw std::runtime_error("USB control write failed: " +
                                     std::string(libusb_error_name(rc)));
        unsigned char buffer[1024];
        rc = libusb_bulk_transfer(handle, 0x81, buffer, sizeof(buffer), &transferred, 5000);
        if (rc)
            throw std::runtime_error("USB control read failed: " +
                                     std::string(libusb_error_name(rc)));
        return parse(std::string(reinterpret_cast<char *>(buffer), transferred));
    }
    void prepare(Json::Value &) override {
        finish();
        try {
            for (auto &read : reads) {
                read.owner = this;
                read.transfer = libusb_alloc_transfer(0);
                if (!read.transfer)
                    throw std::bad_alloc();
                libusb_fill_bulk_transfer(read.transfer, handle, 0x82, read.data.data(),
                                          read.data.size(), completeRead, &read, 0);
                std::lock_guard<std::mutex> lock(readMutex);
                int rc = libusb_submit_transfer(read.transfer);
                if (rc)
                    throw std::runtime_error(libusb_error_name(rc));
                read.submitted = true;
            }
        } catch (...) {
            finish();
            throw;
        }
    }
    void finish() override {
        if (!ctx)
            return;
        {
            std::lock_guard<std::mutex> lock(readMutex);
            for (auto &read : reads)
                if (read.submitted)
                    libusb_cancel_transfer(read.transfer);
        }
        for (;;) {
            bool pending = false;
            {
                std::lock_guard<std::mutex> lock(readMutex);
                for (auto &read : reads)
                    pending |= read.submitted;
            }
            if (!pending)
                break;
            timeval timeout{0, 100000};
            libusb_handle_events_timeout(ctx, &timeout);
        }
        completed.clear();
        for (auto &read : reads) {
            libusb_free_transfer(read.transfer);
            read.transfer = nullptr;
        }
    }
    int receive(uint8_t *data, size_t capacity) override {
        Read *read = nullptr;
        for (unsigned attempt = 0; attempt < 2 && !read; attempt++) {
            {
                std::lock_guard<std::mutex> lock(readMutex);
                if (!completed.empty()) {
                    read = completed.front();
                    completed.pop_front();
                }
            }
            if (!read && attempt == 0) {
                timeval timeout{0, 100000};
                int rc = libusb_handle_events_timeout(ctx, &timeout);
                if (rc && rc != LIBUSB_ERROR_INTERRUPTED)
                    throw std::runtime_error(libusb_error_name(rc));
            }
        }
        if (!read)
            return 0;
        if (read->transfer->status != LIBUSB_TRANSFER_COMPLETED)
            throw std::runtime_error("USB sample transfer failed");
        const size_t n = read->transfer->actual_length;
        if (n > capacity)
            throw std::runtime_error("USB receive buffer too small");
        std::memcpy(data, read->data.data(), n);
        std::lock_guard<std::mutex> lock(readMutex);
        int rc = libusb_submit_transfer(read->transfer);
        if (rc)
            throw std::runtime_error(libusb_error_name(rc));
        read->submitted = true;
        return int(n);
    }
    std::string name() const override { return "usb"; }
};
std::unique_ptr<Transport> makeTransport(const SoapySDR::Kwargs &args) {
    if (args.count("host") || (!args.count("usb") && !args.count("usb_serial")))
        return std::make_unique<Ethernet>(args);
    return std::make_unique<Usb>(args);
}
SoapySDR::KwargsList discover(const SoapySDR::Kwargs &args) {
    if (args.count("host")) {
        auto found = args;
        found["driver"] = "espsdr";
        found["label"] = "ESP-SDR RX (" + args.at("host") + ")";
        return {found};
    }
    SoapySDR::KwargsList result;
    libusb_context *ctx = nullptr;
    if (libusb_init(&ctx))
        return result;
    libusb_device **list = nullptr;
    auto n = libusb_get_device_list(ctx, &list);
    for (ssize_t i = 0; i < n; i++) {
        libusb_device_descriptor d{};
        libusb_get_device_descriptor(list[i], &d);
        if (d.idVendor != usbVid || d.idProduct != usbPid || d.bcdDevice != usbRevision)
            continue;
        libusb_device_handle *h = nullptr;
        if (libusb_open(list[i], &h))
            continue;
        unsigned char s[128]{};
        int len = libusb_get_string_descriptor_ascii(h, d.iSerialNumber, s, sizeof(s));
        libusb_close(h);
        if (len <= 0)
            continue;
        std::string serial(reinterpret_cast<char *>(s));
        if (args.count("usb_serial") && serial != args.at("usb_serial"))
            continue;
        auto found = args;
        found["driver"] = "espsdr";
        found["label"] = "ESP-SDR RX USB " + serial;
        found["usb_serial"] = serial;
        result.push_back(found);
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    return result;
}
} // namespace espsdr
