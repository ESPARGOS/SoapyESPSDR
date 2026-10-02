#include "Transport.hpp"
#include "DcRemoval.hpp"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Version.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <thread>

namespace espsdr {
class Device final : public SoapySDR::Device {
    std::unique_ptr<Transport> transport;
    Json::Value config;
    mutable std::mutex controlMutex;
    std::mutex queueMutex;
    std::condition_variable available;
    std::thread reader;
    std::atomic<bool> active{false};
    std::atomic<uint32_t> epoch{0};
    std::atomic<uint64_t> missing{0}, received{0}, queueDrops{0}, invalid{0};
    std::deque<Packet> queue;
    std::string failure, format;
    bool streamOpen = false, overflow = false;
    Packet current{};
    DcRemoval dcRemoval;
    std::atomic<bool> automaticDc{true};
    bool currentDc = false, dcInitialized = false;
    bool residualDc = true;
    uint32_t dcEpoch = 0;
    std::array<float, payloadBytes> corrected{};
    size_t offset = packetSamples;
    uint64_t expected = 0;
    bool haveExpected = false;
    uint32_t readEpoch = 0;
    static constexpr size_t queueCapacity = 16384;
    static void channel(int direction, size_t ch) {
        if (direction != SOAPY_SDR_RX || ch)
            throw std::runtime_error("ESP-SDR has one RX channel");
    }
    Json::Value request(Json::Value value) {
        std::lock_guard<std::mutex> lock(controlMutex);
        config = transport->request(value);
        epoch = config["epoch"].asUInt();
        return config;
    }
    double configNumber(const char *key) const {
        std::lock_guard<std::mutex> lock(controlMutex);
        return config[key].asDouble();
    }
    void setting(const char *key, double value) {
        if (!std::isfinite(value) || value < 0 || value > UINT32_MAX)
            throw std::runtime_error("Invalid receiver setting");
        Json::Value r;
        r["op"] = "configure";
        r[key] = Json::UInt(value);
        request(r);
    }
    void receiveLoop() noexcept {
        try {
            std::vector<uint8_t> buffer(65536);
            // Cancellation at the end of a previous USB acquisition can leave
            // the endpoint partway through a packet. Recover its initial boundary.
            PacketAssembler assembler(transport->name() == "usb");
            while (active) {
                int bytes = transport->receive(buffer.data(), buffer.size());
                if (bytes <= 0)
                    continue;
                assembler.append(buffer.data(), bytes);
                for (;;) {
                    Packet p;
                    try {
                        if (!assembler.next(p))
                            break;
                    } catch (...) {
                        invalid++;
                        throw;
                    }
                    const uint32_t known = epoch.load();
                    if (p.epoch != known) {
                        if (static_cast<int32_t>(p.epoch - known) <= 0)
                            continue;
                        // A web-interface retune starts a new acquisition epoch.
                        epoch = p.epoch;
                    }
                    std::lock_guard<std::mutex> lock(queueMutex);
                    if (queue.size() == queueCapacity) {
                        queueDrops += packetSamples;
                        continue;
                    }
                    queue.push_back(std::move(p));
                    available.notify_one();
                }
            }
        } catch (const std::exception &e) {
            SoapySDR::logf(SOAPY_SDR_ERROR, "ESP-SDR receive: %s", e.what());
            std::lock_guard<std::mutex> lock(queueMutex);
            failure = e.what();
            available.notify_all();
        }
    }

  public:
    explicit Device(const SoapySDR::Kwargs &args) : transport(makeTransport(args)) {
        auto dc = args.find("dc_offset");
        if (dc != args.end()) {
            if (dc->second != "true" && dc->second != "false")
                throw std::runtime_error("dc_offset must be true or false");
            automaticDc = dc->second == "true";
        }
        Json::Value r;
        r["op"] = "status";
        request(r);
        if (dc == args.end())
            automaticDc = config["dc_correction"].asUInt() != 0;
        auto residual = args.find("dc_residual");
        if (residual != args.end()) {
            if (residual->second != "true" && residual->second != "false")
                throw std::runtime_error("dc_residual must be true or false");
            residualDc = residual->second == "true";
        }
    }
    ~Device() override {
        try {
            if (active)
                deactivateStream(reinterpret_cast<SoapySDR::Stream *>(this));
        } catch (...) {
        }
        if (reader.joinable()) {
            active = false;
            reader.join();
        }
    }
    std::string getDriverKey() const override { return "espsdr"; }
    std::string getHardwareKey() const override { return "ESP32-S31 RX"; }
    SoapySDR::Kwargs getHardwareInfo() const override {
        return {{"transport", transport->name()}, {"protocol", "2"}};
    }
    size_t getNumChannels(int dir) const override { return dir == SOAPY_SDR_RX ? 1 : 0; }
    bool getFullDuplex(int dir, size_t ch) const override {
        channel(dir, ch);
        return false;
    }
    std::vector<std::string> listAntennas(int dir, size_t ch) const override {
        channel(dir, ch);
        return {"RX"};
    }
    void setAntenna(int dir, size_t ch, const std::string &s) override {
        channel(dir, ch);
        if (s != "RX")
            throw std::runtime_error("Unknown antenna");
    }
    std::string getAntenna(int dir, size_t ch) const override {
        channel(dir, ch);
        return "RX";
    }
    std::vector<std::string> getStreamFormats(int dir, size_t ch) const override {
        channel(dir, ch);
        return {SOAPY_SDR_CS8, SOAPY_SDR_CS16, SOAPY_SDR_CF32};
    }
    std::string getNativeStreamFormat(int dir, size_t ch, double &scale) const override {
        channel(dir, ch);
        scale = 128;
        return SOAPY_SDR_CS8;
    }
    bool hasDCOffsetMode(int dir, size_t ch) const override {
        channel(dir, ch);
        return true;
    }
    void setDCOffsetMode(int dir, size_t ch, bool automatic) override {
        channel(dir, ch);
        setting("dc_correction", automatic ? 1 : 0);
        automaticDc = automatic;
    }
    bool getDCOffsetMode(int dir, size_t ch) const override {
        channel(dir, ch);
        return automaticDc;
    }
    std::vector<std::string> listGains(int dir, size_t ch) const override {
        channel(dir, ch);
        return {"RF"};
    }
    void setGain(int dir, size_t ch, double value) override {
        channel(dir, ch);
        setting("gain", std::round(value));
    }
    void setGain(int dir, size_t ch, const std::string &name, double value) override {
        if (name != "RF")
            throw std::runtime_error("Unknown gain");
        setGain(dir, ch, value);
    }
    double getGain(int dir, size_t ch) const override {
        channel(dir, ch);
        return configNumber("gain");
    }
    double getGain(int dir, size_t ch, const std::string &name) const override {
        if (name != "RF")
            throw std::runtime_error("Unknown gain");
        return getGain(dir, ch);
    }
    SoapySDR::Range getGainRange(int dir, size_t ch) const override {
        channel(dir, ch);
        return {0, configNumber("gain_max"), 1};
    }
    SoapySDR::Range getGainRange(int dir, size_t ch, const std::string &name) const override {
        if (name != "RF")
            throw std::runtime_error("Unknown gain");
        return getGainRange(dir, ch);
    }
    std::vector<std::string> listFrequencies(int dir, size_t ch) const override {
        channel(dir, ch);
        return {"RF"};
    }
    void setFrequency(int dir, size_t ch, double hz, const SoapySDR::Kwargs &) override {
        channel(dir, ch);
        setting("frequency", std::round(hz / 1e6) * 1e6);
    }
    void setFrequency(int dir, size_t ch, const std::string &name, double hz,
                      const SoapySDR::Kwargs &args) override {
        if (name != "RF")
            throw std::runtime_error("Unknown frequency component");
        setFrequency(dir, ch, hz, args);
    }
    double getFrequency(int dir, size_t ch) const override {
        channel(dir, ch);
        return configNumber("frequency");
    }
    double getFrequency(int dir, size_t ch, const std::string &name) const override {
        if (name != "RF")
            throw std::runtime_error("Unknown frequency component");
        return getFrequency(dir, ch);
    }
    SoapySDR::RangeList getFrequencyRange(int dir, size_t ch) const override {
        channel(dir, ch);
        return {{2300e6, 2800e6, 1e6}};
    }
    SoapySDR::RangeList getFrequencyRange(int dir, size_t ch,
                                          const std::string &name) const override {
        if (name != "RF")
            throw std::runtime_error("Unknown frequency component");
        return getFrequencyRange(dir, ch);
    }
    void setSampleRate(int dir, size_t ch, double rate) override {
        channel(dir, ch);
        auto rates = listSampleRates(dir, ch);
        if (std::find(rates.begin(), rates.end(), rate) == rates.end())
            throw std::runtime_error("Unsupported rate");
        setting("rate", rate);
    }
    double getSampleRate(int dir, size_t ch) const override {
        channel(dir, ch);
        return configNumber("rate");
    }
    std::vector<double> listSampleRates(int dir, size_t ch) const override {
        channel(dir, ch);
        if (transport->name() == "usb")
            return {4e6, 8e6, 16e6, 20e6};
        return {4e6, 8e6, 16e6, 20e6, 40e6};
    }
    SoapySDR::RangeList getSampleRateRange(int dir, size_t ch) const override {
        SoapySDR::RangeList r;
        for (auto rate : listSampleRates(dir, ch))
            r.emplace_back(rate, rate);
        return r;
    }
    void setBandwidth(int dir, size_t ch, double bw) override {
        channel(dir, ch);
        setting("bandwidth", std::round(bw / 1e6) * 1e6);
    }
    double getBandwidth(int dir, size_t ch) const override {
        channel(dir, ch);
        return configNumber("bandwidth");
    }
    SoapySDR::RangeList getBandwidthRange(int dir, size_t ch) const override {
        channel(dir, ch);
        return {{0, 0}, {13e6, 54e6, 1e6}};
    }
    SoapySDR::Stream *setupStream(int dir, const std::string &fmt,
                                  const std::vector<size_t> &channels,
                                  const SoapySDR::Kwargs &) override {
        channel(dir, channels.empty() ? 0 : channels[0]);
        if (channels.size() > 1 || streamOpen)
            throw std::runtime_error("Only one RX stream is supported");
        auto formats = getStreamFormats(dir, 0);
        if (std::find(formats.begin(), formats.end(), fmt) == formats.end())
            throw std::runtime_error("Unsupported sample format");
        format = fmt;
        streamOpen = true;
        return reinterpret_cast<SoapySDR::Stream *>(this);
    }
    void closeStream(SoapySDR::Stream *s) override {
        check(s);
        if (active)
            deactivateStream(s);
        streamOpen = false;
    }
    void check(SoapySDR::Stream *s) const {
        if (!streamOpen || s != reinterpret_cast<const SoapySDR::Stream *>(this))
            throw std::runtime_error("Invalid stream");
    }
    size_t getStreamMTU(SoapySDR::Stream *s) const override {
        check(s);
        return packetSamples;
    }
    int activateStream(SoapySDR::Stream *s, int flags = 0, long long timeNs = 0,
                       size_t count = 0) override {
        check(s);
        (void)timeNs;
        if (flags || count)
            return SOAPY_SDR_NOT_SUPPORTED;
        if (active)
            return SOAPY_SDR_STREAM_ERROR;
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            queue.clear();
            failure.clear();
        }
        offset = packetSamples;
        dcInitialized = false;
        haveExpected = false;
        overflow = false;
        missing = 0;
        received = 0;
        queueDrops = 0;
        invalid = 0;
        Json::Value r;
        r["op"] = "start";
        r["dc_correction"] = automaticDc.load() ? 1 : 0;
        bool startAttempted = false;
        try {
            transport->prepare(r);
            startAttempted = true;
            request(r);
            active = true;
            reader = std::thread(&Device::receiveLoop, this);
        } catch (const std::exception &e) {
            active = false;
            // A lost reply does not mean the device rejected the start. Stop
            // even when request() failed, so an orphaned UDP stream cannot
            // keep saturating the network after activation has failed.
            if (startAttempted) {
                Json::Value stop;
                stop["op"] = "stop";
                try {
                    request(stop);
                } catch (const std::exception &cleanup) {
                    SoapySDR::logf(SOAPY_SDR_WARNING, "ESP-SDR start cleanup: %s", cleanup.what());
                }
            }
            transport->finish();
            SoapySDR::logf(SOAPY_SDR_ERROR, "ESP-SDR activation failed: %s", e.what());
            return SOAPY_SDR_STREAM_ERROR;
        }
        return 0;
    }
    int deactivateStream(SoapySDR::Stream *s, int flags = 0, long long timeNs = 0) override {
        check(s);
        (void)timeNs;
        if (flags)
            return SOAPY_SDR_NOT_SUPPORTED;
        if (!active && !reader.joinable())
            return 0;
        Json::Value r;
        r["op"] = "stop";
        std::exception_ptr error;
        try {
            // Keep draining USB/UDP until the producer has stopped. Joining
            // first leaves the device capturing into an undrained ring.
            request(r);
        } catch (...) {
            error = std::current_exception();
        }
        active = false;
        available.notify_all();
        if (reader.joinable())
            reader.join();
        transport->finish();
        if (error) {
            try {
                std::rethrow_exception(error);
            } catch (const std::exception &e) {
                SoapySDR::logf(SOAPY_SDR_ERROR, "ESP-SDR deactivation failed: %s", e.what());
            }
            return SOAPY_SDR_STREAM_ERROR;
        }
        return 0;
    }
    int readStream(SoapySDR::Stream *s, void *const *buffers, size_t count, int &flags,
                   long long &timeNs, long timeoutUs = 100000) override {
        check(s);
        flags = 0;
        if (!active)
            return SOAPY_SDR_STREAM_ERROR;
        if (!count)
            return 0;
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);
        while (offset == packetSamples || current.epoch != epoch.load()) {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (!available.wait_until(
                    lock, deadline, [&] { return !queue.empty() || !failure.empty() || !active; }))
                return SOAPY_SDR_TIMEOUT;
            if (!failure.empty() || !active)
                return SOAPY_SDR_STREAM_ERROR;
            current = std::move(queue.front());
            queue.pop_front();
            // DC estimation and sample conversion must not hold up the
            // transport reader while it replenishes asynchronous USB reads.
            lock.unlock();
            offset = 0;
            if (current.epoch != epoch.load()) {
                offset = packetSamples;
                continue;
            }
            if (readEpoch != current.epoch) {
                haveExpected = true;
                expected = 0;
                readEpoch = current.epoch;
            }
            if (haveExpected && current.sample < expected) {
                offset = packetSamples;
                continue;
            }
            if (haveExpected && current.sample != expected) {
                missing += current.sample - expected;
                overflow = true;
            }
            haveExpected = true;
            expected = current.sample + packetSamples;
            const bool enabled = automaticDc.load() && residualDc;
            if (!dcInitialized || dcEpoch != current.epoch || currentDc != enabled || overflow) {
                dcRemoval.reset(current.rate);
                dcEpoch = current.epoch;
                dcInitialized = true;
            }
            currentDc = enabled;
            if (currentDc)
                dcRemoval.process(current.iq.data(), corrected.data(), packetSamples);
        }
        if (overflow) {
            overflow = false;
            return SOAPY_SDR_OVERFLOW;
        }
        size_t n = std::min(count, packetSamples - offset);
        const int8_t *in = current.iq.data() + 2 * offset;
        if (currentDc) {
            const float *values = corrected.data() + 2 * offset;
            if (format == SOAPY_SDR_CF32) {
                auto out = static_cast<float *>(buffers[0]);
                for (size_t i = 0; i < 2 * n; ++i)
                    out[i] = values[i] / 128.0f;
            } else if (format == SOAPY_SDR_CS16) {
                auto out = static_cast<int16_t *>(buffers[0]);
                for (size_t i = 0; i < 2 * n; ++i)
                    out[i] = static_cast<int16_t>(
                        std::clamp(std::round(values[i] * 256), -32768.0f, 32767.0f));
            } else {
                auto out = static_cast<int8_t *>(buffers[0]);
                for (size_t i = 0; i < 2 * n; ++i)
                    out[i] = static_cast<int8_t>(std::clamp(std::round(values[i]), -128.0f, 127.0f));
            }
        } else if (format == SOAPY_SDR_CS8)
            std::memcpy(buffers[0], in, 2 * n);
        else if (format == SOAPY_SDR_CS16) {
            auto out = static_cast<int16_t *>(buffers[0]);
            for (size_t i = 0; i < 2 * n; i++)
                out[i] = int16_t(in[i]) * 256;
        } else {
            auto out = static_cast<float *>(buffers[0]);
            for (size_t i = 0; i < 2 * n; i++)
                out[i] = float(in[i]) / 128.0f;
        }
        // Packet time is truncated to microseconds; recover the fractional
        // sample position before adding this application's partial-read offset.
        timeNs = static_cast<long long>(current.timeUs) * 1000 +
                 static_cast<long long>(current.sample % (current.rate / 1000000u) + offset) *
                     1000000000 / current.rate;
        flags = SOAPY_SDR_HAS_TIME;
        offset += n;
        received += n;
        return int(n);
    }
    bool hasHardwareTime(const std::string &what = "") const override { return what.empty(); }
    long long getHardwareTime(const std::string &what = "") const override {
        if (!what.empty())
            throw std::runtime_error("Unknown clock");
        Json::Value r;
        r["op"] = "status";
        return const_cast<Device *>(this)->request(r)["time_us"].asInt64() * 1000;
    }
    std::vector<std::string> listSensors() const override {
        return {"rx_samples", "rx_missing_samples", "rx_queue_drops", "rx_invalid_packets"};
    }
    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override {
        auto keys = listSensors();
        if (std::find(keys.begin(), keys.end(), key) == keys.end())
            throw std::runtime_error("Unknown sensor");
        SoapySDR::ArgInfo info;
        info.key = key;
        info.name = key;
        info.type = SoapySDR::ArgInfo::INT;
        info.value = "0";
        return info;
    }
    std::string readSensor(const std::string &key) const override {
        if (key == "rx_samples")
            return std::to_string(received.load());
        if (key == "rx_missing_samples")
            return std::to_string(missing.load());
        if (key == "rx_queue_drops")
            return std::to_string(queueDrops.load());
        if (key == "rx_invalid_packets")
            return std::to_string(invalid.load());
        throw std::runtime_error("Unknown sensor");
    }
};
static SoapySDR::Device *make(const SoapySDR::Kwargs &args) { return new Device(args); }
static SoapySDR::Registry registration("espsdr", discover, make, SOAPY_SDR_ABI_VERSION);
} // namespace espsdr
