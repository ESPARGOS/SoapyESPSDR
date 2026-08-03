#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    if (argc < 4 || argc > 6) {
        std::cerr << "usage: espsdr_live_rate_test HOST_OR_DEVICE_ARGS RATE_HZ SECONDS [CS16|CF32] [CYCLE_TOTAL]\n";
        return 2;
    }
    const std::string selector = argv[1];
    const std::string args = "driver=espsdr," +
        (selector.find('=') == std::string::npos ? "host=" + selector : selector);
    const double rate = std::stod(argv[2]);
    const double seconds = std::stod(argv[3]);
    const std::string format = argc >= 5 ? argv[4] : SOAPY_SDR_CS16;
    const std::string cycleTotal = argc == 6 ? argv[5] : "1";
    SoapySDR::Device *device = SoapySDR::Device::make(args);
    if (device == nullptr) {
        std::cerr << "failed to make device\n";
        return 1;
    }
    int result = 0;
    SoapySDR::Stream *stream = nullptr;
    try {
        device->setSampleRate(SOAPY_SDR_RX, 0, rate);
        device->writeSetting("cycle_total", cycleTotal);
        device->writeSetting("cycle_stream", "1");
        stream = device->setupStream(SOAPY_SDR_RX, format);
        if (device->activateStream(stream) != 0) throw std::runtime_error("activateStream failed");
        std::vector<float> samples(16384 * 2);
        void *buffers[] = {samples.data()};
        uint64_t received = 0;
        uint64_t overflows = 0;
        uint64_t timeouts = 0;
        const auto start = std::chrono::steady_clock::now();
        const auto deadline = start + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < deadline) {
            int flags = 0;
            long long timeNs = 0;
            const int count = device->readStream(stream, buffers, 16384, flags, timeNs, 250000);
            if (count > 0) received += static_cast<unsigned>(count);
            else if (count == SOAPY_SDR_OVERFLOW) ++overflows;
            else if (count == SOAPY_SDR_TIMEOUT) ++timeouts;
            else throw std::runtime_error("readStream failed: " + std::to_string(count));
        }
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::cout << "format=" << format << " rate_hz=" << rate << " elapsed_s=" << elapsed
                  << " samples=" << received << " measured_msa_s=" << received / elapsed / 1e6
                  << " overflows=" << overflows << " timeouts=" << timeouts;
        for (const auto &sensor : device->listSensors()) {
            std::cout << ' ' << sensor << '=' << device->readSensor(sensor);
        }
        std::cout << '\n';
        // The interval includes firmware stream startup, so allow a small startup
        // deficit while still requiring continuity once packets arrive.
        if (overflows != 0 || received < rate * elapsed * 0.95) result = 1;
        device->deactivateStream(stream);
        device->closeStream(stream);
        stream = nullptr;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        result = 1;
        if (stream != nullptr) {
            try { device->deactivateStream(stream); } catch (...) {}
            try { device->closeStream(stream); } catch (...) {}
        }
    }
    SoapySDR::Device::unmake(device);
    return result;
}
