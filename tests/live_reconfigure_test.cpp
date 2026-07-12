#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "usage: espsdr_live_reconfigure_test HOST\n";
        return 2;
    }
    SoapySDR::Device *device = SoapySDR::Device::make(
        "driver=espsdr,host=" + std::string(argv[1]));
    if (device == nullptr) return 1;
    const double oldFrequency = device->getFrequency(SOAPY_SDR_RX, 0);
    const double oldBandwidth = device->getBandwidth(SOAPY_SDR_RX, 0);
    const double oldGain = device->getGain(SOAPY_SDR_RX, 0);
    const double oldRate = device->getSampleRate(SOAPY_SDR_RX, 0);
    const bool oldAgc = device->getGainMode(SOAPY_SDR_RX, 0);
    SoapySDR::Stream *stream = nullptr;
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> samples{0};
    std::atomic<uint64_t> overflows{0};
    std::atomic<uint64_t> timeouts{0};
    std::thread reader;
    int result = 0;
    try {
        device->setSampleRate(SOAPY_SDR_RX, 0, 8e6);
        stream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
        if (device->activateStream(stream) != 0) throw std::runtime_error("activateStream failed");
        reader = std::thread([&]() {
            std::vector<int16_t> data(16384 * 2);
            void *buffers[] = {data.data()};
            while (!stop) {
                int flags = 0;
                long long timeNs = 0;
                const int count = device->readStream(stream, buffers, 16384, flags, timeNs, 250000);
                if (count > 0) samples += static_cast<unsigned>(count);
                else if (count == SOAPY_SDR_OVERFLOW) ++overflows;
                else if (count == SOAPY_SDR_TIMEOUT) ++timeouts;
                else stop = true;
            }
        });
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const auto change = [&](const char *name, const std::function<void()> &operation) {
            const uint64_t before = samples.load();
            const auto start = std::chrono::steady_clock::now();
            operation();
            const double applySeconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            const auto resumeDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (samples.load() <= before + 1024 && std::chrono::steady_clock::now() < resumeDeadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            const bool resumed = samples.load() > before + 1024;
            std::cout << name << " apply_s=" << applySeconds << " resumed=" << (resumed ? "yes" : "no") << '\n';
            if (!resumed) throw std::runtime_error(std::string(name) + " did not resume streaming");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        };

        change("frequency", [&]() { device->setFrequency(SOAPY_SDR_RX, 0, 2437125000.0); });
        change("bandwidth", [&]() { device->setBandwidth(SOAPY_SDR_RX, 0, 21e6); });
        change("agc", [&]() { device->setGainMode(SOAPY_SDR_RX, 0, true); });
        change("manual_gain", [&]() { device->setGain(SOAPY_SDR_RX, 0, 40); });
        change("sample_rate_16m", [&]() { device->setSampleRate(SOAPY_SDR_RX, 0, 16e6); });
        change("sample_rate_8m", [&]() { device->setSampleRate(SOAPY_SDR_RX, 0, 8e6); });

        stop = true;
        reader.join();
        std::cout << "samples=" << samples.load() << " overflows=" << overflows.load()
                  << " timeouts=" << timeouts.load();
        for (const auto &sensor : device->listSensors()) {
            std::cout << ' ' << sensor << '=' << device->readSensor(sensor);
        }
        std::cout << '\n';
        device->deactivateStream(stream);
        device->closeStream(stream);
        stream = nullptr;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        result = 1;
        stop = true;
        if (reader.joinable()) reader.join();
        if (stream != nullptr) {
            try { device->deactivateStream(stream); } catch (...) {}
            try { device->closeStream(stream); } catch (...) {}
            stream = nullptr;
        }
    }
    try {
        device->setFrequency(SOAPY_SDR_RX, 0, oldFrequency);
        device->setBandwidth(SOAPY_SDR_RX, 0, oldBandwidth);
        device->setGain(SOAPY_SDR_RX, 0, oldGain);
        device->setGainMode(SOAPY_SDR_RX, 0, oldAgc);
        const bool oldRateSupported = oldRate == 8e6 || oldRate == 16e6 || oldRate == 20e6;
        device->setSampleRate(SOAPY_SDR_RX, 0, oldRateSupported ? oldRate : 8e6);
    } catch (const std::exception &error) {
        std::cerr << "restore failed: " << error.what() << '\n';
        result = 1;
    }
    SoapySDR::Device::unmake(device);
    return result;
}
