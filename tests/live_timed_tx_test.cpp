#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char **argv)
{
    if (argc < 2 || argc > 4) {
        std::cerr << "usage: espsdr_live_timed_tx_test HOST_OR_DEVICE_ARGS [SAMPLES [RATE_HZ]]\n";
        return 2;
    }
    const std::string selector = argv[1];
    SoapySDR::Device *device = SoapySDR::Device::make(
        "driver=espsdr," +
        (selector.find('=') == std::string::npos ? "host=" + selector
                                                 : selector));
    if (device == nullptr) return 1;

    SoapySDR::Stream *stream = nullptr;
    int result = 0;
    try {
        if (!device->hasHardwareTime())
            throw std::runtime_error("hardware time is not advertised");
        device->setFrequency(SOAPY_SDR_TX, 0, 2.38e9);
        device->setBandwidth(SOAPY_SDR_TX, 0, 20e6);
        const double sampleRate = argc == 4 ? std::stod(argv[3]) : 4e6;
        device->setSampleRate(SOAPY_SDR_TX, 0, sampleRate);
        device->setGain(SOAPY_SDR_TX, 0, 4);

        const std::size_t count = argc >= 3
            ? static_cast<std::size_t>(std::stoull(argv[2])) : 16383u;
        constexpr double pi = 3.14159265358979323846;
        std::vector<int16_t> samples(count * 2);
        for (std::size_t i = 0; i < count; ++i) {
            samples[2 * i] = static_cast<int16_t>(
                std::lround(24000.0 * std::cos(2.0 * pi * i / 32.0)));
            samples[2 * i + 1] = static_cast<int16_t>(
                std::lround(24000.0 * std::sin(2.0 * pi * i / 32.0)));
        }

        stream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
        if (count == 0 || count > device->getStreamMTU(stream))
            throw std::runtime_error("sample count is outside the TX MTU");
        if (device->activateStream(stream) != 0)
            throw std::runtime_error("activateStream failed");
        const long long leadTimeNs = count > 200'000u
            ? 3'000'000'000LL : 1'000'000'000LL;
        const long long requested = device->getHardwareTime() + leadTimeNs;
        const auto hostStart = std::chrono::steady_clock::now();
        std::size_t totalWritten = 0;
        while (totalWritten < count) {
            const std::size_t fragment = std::min<std::size_t>(
                4096, count - totalWritten);
            const void *buffers[] = {samples.data() + totalWritten * 2};
            int flags = totalWritten == 0 ? SOAPY_SDR_HAS_TIME : 0;
            if (totalWritten + fragment == count)
                flags |= SOAPY_SDR_END_BURST;
            const int written = device->writeStream(
                stream, buffers, fragment, flags,
                totalWritten == 0 ? requested : 0, 3'000'000);
            if (written != static_cast<int>(fragment))
                throw std::runtime_error(std::string("timed write failed: ") +
                                         SoapySDR::errToStr(written));
            totalWritten += static_cast<std::size_t>(written);
        }
        const double blockedSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - hostStart).count();

        std::size_t channels = 0;
        int statusFlags = 0;
        long long actual = 0;
        const int status = device->readStreamStatus(
            stream, channels, statusFlags, actual, 100'000);
        if (status != 0 || channels != 1u ||
            (statusFlags & (SOAPY_SDR_END_BURST | SOAPY_SDR_HAS_TIME)) !=
                (SOAPY_SDR_END_BURST | SOAPY_SDR_HAS_TIME))
            throw std::runtime_error("timed completion status is missing");
        const long long error = actual - requested;
        const long long firmwareError = std::stoll(
            device->readSensor("tx_replay_start_error_ns"));
        if (actual <= 0 || std::llabs(error) > 1'000'000 ||
            error != firmwareError)
            throw std::runtime_error("timed TX start error exceeds 1 ms");
        if (blockedSeconds < leadTimeNs / 1e9 - 0.5 ||
            blockedSeconds > leadTimeNs / 1e9 + 1.5)
            throw std::runtime_error("timed write did not wait for its deadline");

        std::vector<int16_t> stagingSamples(16383u * 2u);
        for (std::size_t i = 0; i < 16383u; ++i) {
            stagingSamples[i * 2] = samples[(i % count) * 2];
            stagingSamples[i * 2 + 1] = samples[(i % count) * 2 + 1];
        }
        const void *stagingBuffers[] = {stagingSamples.data()};
        /* This deadline is valid when writeStream enters, but Ethernet
         * staging necessarily takes longer. Firmware must suppress RF and
         * the host must report TIME_ERROR instead of transmitting late. */
        int stagingFlags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
        const long long stagingDeadline =
            device->getHardwareTime() + 1'000'000LL;
        const int stagingExpired = device->writeStream(
            stream, stagingBuffers, 16383, stagingFlags, stagingDeadline,
            3'000'000);
        const std::string stagingBuffered =
            device->readSensor("tx_buffered_samples");
        const std::string stagingMissed =
            device->readSensor("tx_replay_deadline_missed");
        const std::string stagingActual =
            device->readSensor("tx_replay_actual_start_time_ns");
        if (stagingExpired != SOAPY_SDR_TIME_ERROR ||
            stagingBuffered != "0" || stagingMissed != "true" ||
            stagingActual != "0")
            throw std::runtime_error(
                "staging-expired TX was not suppressed cleanly: result=" +
                std::to_string(stagingExpired) + " buffered=" +
                stagingBuffered + " missed=" + stagingMissed +
                " actual=" + stagingActual);

        const void *lateBuffers[] = {samples.data()};
        int lateFlags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
        const int late = device->writeStream(
            stream, lateBuffers, 1, lateFlags,
            device->getHardwareTime() - 1, 100'000);
        if (late != SOAPY_SDR_TIME_ERROR ||
            device->readSensor("tx_buffered_samples") != "0")
            throw std::runtime_error("expired TX deadline was not rejected cleanly");

        int firstFlags = SOAPY_SDR_HAS_TIME;
        const long long fragmentDeadline =
            device->getHardwareTime() + 50'000'000;
        const int first = device->writeStream(
            stream, lateBuffers, 1, firstFlags, fragmentDeadline, 100'000);
        if (first != 1)
            throw std::runtime_error("timed first fragment was not buffered");
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        int finalFlags = SOAPY_SDR_END_BURST;
        const int final = device->writeStream(
            stream, lateBuffers, 1, finalFlags, 0, 100'000);
        if (final != SOAPY_SDR_TIME_ERROR ||
            device->readSensor("tx_buffered_samples") != "0")
            throw std::runtime_error(
                "expired fragmented TX was not discarded cleanly");

        std::cout << "SoapySDR timed TX: requested=" << requested
                  << " actual=" << actual << " error_ns=" << error
                  << " rate_hz=" << sampleRate
                  << " samples=" << count
                  << " write_block_s=" << blockedSeconds << ": OK\n";
        if (device->deactivateStream(stream) != 0)
            throw std::runtime_error("deactivateStream failed");
        device->closeStream(stream);
        stream = nullptr;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    if (stream != nullptr) {
        try { device->deactivateStream(stream); } catch (...) {}
        device->closeStream(stream);
    }
    SoapySDR::Device::unmake(device);
    return result;
}
