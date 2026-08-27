#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>

#include <cmath>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    if (argc < 2 || argc > 3) {
        std::cerr << "usage: espsdr_live_tx_test HOST_OR_DEVICE_ARGS [SAMPLES]\n";
        return 2;
    }
    const std::string selector = argv[1];
    SoapySDR::Device *device = SoapySDR::Device::make(
        "driver=espsdr," +
        (selector.find('=') == std::string::npos ? "host=" + selector : selector));
    if (device == nullptr) return 1;

    int result = 0;
    const double oldFrequency = device->getFrequency(SOAPY_SDR_TX, 0);
    const double oldBandwidth = device->getBandwidth(SOAPY_SDR_TX, 0);
    // Preserve the expert raw code.  It may intentionally be outside the
    // characterized monotonic dB table, in which case getGain(TX) is NaN.
    const std::string oldGainCode = device->readSetting("tx_gain_code");
    SoapySDR::Stream *stream = nullptr;
    try {
        if (device->getNumChannels(SOAPY_SDR_TX) != 1)
            throw std::runtime_error("TX channel is not advertised");
        device->setFrequency(SOAPY_SDR_TX, 0, 2.38e9);
        device->setBandwidth(SOAPY_SDR_TX, 0, 20e6);
        device->setSampleRate(SOAPY_SDR_TX, 0, 20e6);
        device->setGain(SOAPY_SDR_TX, 0, 4);

        const std::size_t count = argc == 3
                                      ? static_cast<std::size_t>(std::stoull(argv[2]))
                                      : 16383u;
        if (count == 0) throw std::runtime_error("sample count must be nonzero");
        constexpr double pi = 3.14159265358979323846;
        std::vector<int16_t> samples(count * 2);
        for (std::size_t i = 0; i < count; ++i) {
            samples[i * 2] = static_cast<int16_t>(
                std::lround(24000.0 * std::cos(2.0 * pi * i / 32.0)));
            samples[i * 2 + 1] = static_cast<int16_t>(
                std::lround(24000.0 * std::sin(2.0 * pi * i / 32.0)));
        }
        stream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
        if (device->getStreamMTU(stream) < count)
            throw std::runtime_error("sample count exceeds the TX stream MTU");
        const int activated = device->activateStream(stream);
        if (activated != 0) throw std::runtime_error("activateStream failed");
        const auto writeStart = std::chrono::steady_clock::now();
        std::size_t totalWritten = 0;
        while (totalWritten < count) {
            const std::size_t fragment = std::min<std::size_t>(
                4096u, count - totalWritten);
            const void *buffers[] = {
                samples.data() + totalWritten * 2u};
            int flags = totalWritten + fragment == count
                            ? SOAPY_SDR_END_BURST
                            : 0;
            const int written = device->writeStream(
                stream, buffers, fragment, flags, 0, 2'000'000);
            if (written != static_cast<int>(fragment))
                throw std::runtime_error(std::string("writeStream failed: ") +
                                         SoapySDR::errToStr(written));
            totalWritten += static_cast<std::size_t>(written);
            const std::size_t buffered = static_cast<std::size_t>(
                std::stoull(device->readSensor("tx_buffered_samples")));
            const std::size_t expectedBuffered =
                totalWritten == count ? 0u : totalWritten;
            if (buffered != expectedBuffered)
                throw std::runtime_error("TX aggregation occupancy mismatch");
        }
        const double writeSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - writeStart).count();
        std::size_t statusChannels = 0;
        int statusFlags = 0;
        long long statusTime = 0;
        const int status = device->readStreamStatus(
            stream, statusChannels, statusFlags, statusTime, 100'000);
        if (status != 0 || statusChannels != 1u ||
            (statusFlags & SOAPY_SDR_END_BURST) == 0)
            throw std::runtime_error("TX burst-completion status missing");
        if (device->deactivateStream(stream) != 0)
            throw std::runtime_error("deactivateStream failed");
        device->closeStream(stream);
        stream = nullptr;
        std::cout << "SoapySDR fragmented TX burst: " << totalWritten
                  << " samples at 20 MSa/s, 2380 MHz in " << writeSeconds
                  << " s: OK\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    if (stream != nullptr) {
        try { device->deactivateStream(stream); } catch (...) {}
        device->closeStream(stream);
    }
    try {
        device->setFrequency(SOAPY_SDR_TX, 0, oldFrequency);
        device->setBandwidth(SOAPY_SDR_TX, 0, oldBandwidth);
        device->writeSetting("tx_gain_code", oldGainCode);
    } catch (const std::exception &error) {
        std::cerr << "restore failed: " << error.what() << '\n';
        result = 1;
    }
    SoapySDR::Device::unmake(device);
    return result;
}
