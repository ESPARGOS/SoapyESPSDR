#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    if (argc < 2 || argc > 4) {
        std::cerr << "usage: espsdr_live_agc_test HOST_OR_DEVICE_ARGS "
                     "[RATE_HZ] [SECONDS]\n";
        return 2;
    }
    const std::string selector = argv[1];
    SoapySDR::Device *device = SoapySDR::Device::make(
        "driver=espsdr," +
        (selector.find('=') == std::string::npos ? "host=" + selector
                                                 : selector));
    if (device == nullptr) return 1;
    const double rate = argc >= 3 ? std::stod(argv[2]) : 8e6;
    const double seconds = argc >= 4 ? std::stod(argv[3]) : 3.0;

    const bool oldAgc = device->getGainMode(SOAPY_SDR_RX, 0);
    const double oldGain = device->getGain(SOAPY_SDR_RX, 0);
    const double oldRate = device->getSampleRate(SOAPY_SDR_RX, 0);
    SoapySDR::Stream *stream = nullptr;
    int result = 0;
    try {
        device->setSampleRate(SOAPY_SDR_RX, 0, rate);
        device->setGainMode(SOAPY_SDR_RX, 0, true);
        stream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
        if (device->activateStream(stream) != 0)
            throw std::runtime_error("activateStream failed");

        std::vector<int16_t> samples(4096 * 2);
        void *buffers[] = {samples.data()};
        uint64_t received = 0;
        uint64_t overflows = 0;
        uint64_t timeouts = 0;
        uint32_t gain = 0;
        uint32_t peak = 0;
        uint32_t changes = 0;
        bool active = false;
        long long slowestSensorReadUs = 0;
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < deadline) {
            int flags = 0;
            long long timeNs = 0;
            const int count = device->readStream(
                stream, buffers, 4096, flags, timeNs, 250000);
            if (count == SOAPY_SDR_OVERFLOW) {
                ++overflows;
                continue;
            }
            if (count == SOAPY_SDR_TIMEOUT) {
                ++timeouts;
                continue;
            }
            if (count < 0)
                throw std::runtime_error("readStream failed: " +
                                         std::to_string(count));
            received += static_cast<uint64_t>(count);
            const auto before = std::chrono::steady_clock::now();
            active = device->readSensor("rx_agc_active") == "true";
            gain = static_cast<uint32_t>(
                std::stoul(device->readSensor("rx_agc_current_gain")));
            peak = static_cast<uint32_t>(
                std::stoul(device->readSensor("rx_agc_robust_peak")));
            changes = static_cast<uint32_t>(
                std::stoul(device->readSensor("rx_agc_gain_changes")));
            const auto elapsed = std::chrono::duration_cast<
                std::chrono::microseconds>(std::chrono::steady_clock::now() -
                                           before).count();
            if (elapsed > slowestSensorReadUs) slowestSensorReadUs = elapsed;
        }
        std::cout << "rate_hz=" << rate << " seconds=" << seconds
                  << " samples=" << received << " overflows=" << overflows
                  << " timeouts=" << timeouts << " agc_active=" << active
                  << " current_gain_db=" << gain
                  << " robust_peak=" << peak
                  << " gain_changes=" << changes
                  << " slowest_sensor_group_us=" << slowestSensorReadUs
                  << '\n';
        if (received < rate * seconds * 0.9 || overflows != 0u || !active ||
            peak == 0u ||
            gain > device->getGainRange(SOAPY_SDR_RX, 0).maximum() ||
            changes == 0u || slowestSensorReadUs > 20'000)
            throw std::runtime_error("live in-band AGC telemetry validation failed");
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    if (stream != nullptr) {
        try { device->deactivateStream(stream); } catch (...) { result = 1; }
        try { device->closeStream(stream); } catch (...) { result = 1; }
    }
    try {
        device->setSampleRate(SOAPY_SDR_RX, 0, oldRate);
        device->setGain(SOAPY_SDR_RX, 0, oldGain);
        device->setGainMode(SOAPY_SDR_RX, 0, oldAgc);
    } catch (const std::exception &error) {
        std::cerr << "restore failed: " << error.what() << '\n';
        result = 1;
    }
    SoapySDR::Device::unmake(device);
    return result;
}
