#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>

#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::size_t receiveSome(SoapySDR::Device *device, SoapySDR::Stream *stream)
{
    if (device->activateStream(stream) != 0)
        throw std::runtime_error("RX activateStream failed");
    std::vector<int16_t> samples(8192 * 2);
    void *buffers[] = {samples.data()};
    std::size_t received = 0;
    for (unsigned attempt = 0; attempt < 20 && received < 8192; ++attempt) {
        int flags = 0;
        long long timeNs = 0;
        const int result = device->readStream(stream, buffers, 8192, flags,
                                              timeNs, 500'000);
        if (result > 0) received += static_cast<std::size_t>(result);
        else if (result != SOAPY_SDR_TIMEOUT && result != SOAPY_SDR_OVERFLOW)
            throw std::runtime_error(std::string("RX readStream failed: ") +
                                     SoapySDR::errToStr(result));
    }
    if (device->deactivateStream(stream) != 0)
        throw std::runtime_error("RX deactivateStream failed");
    if (received == 0) throw std::runtime_error("RX returned no samples");
    return received;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2 || argc > 4) {
        std::cerr << "usage: espsdr_live_tdd_test HOST_OR_DEVICE_ARGS "
                     "[CYCLES [TX_SAMPLES]]\n";
        return 2;
    }
    const unsigned cycles = argc >= 3
        ? static_cast<unsigned>(std::stoul(argv[2])) : 5u;
    const std::size_t count = argc >= 4
        ? static_cast<std::size_t>(std::stoull(argv[3])) : 16383u;
    if (cycles == 0 || count == 0) {
        std::cerr << "cycles and TX samples must be nonzero\n";
        return 2;
    }
    const std::string selector = argv[1];
    SoapySDR::Device *device = SoapySDR::Device::make(
        "driver=espsdr," +
        (selector.find('=') == std::string::npos ? "host=" + selector : selector));
    if (device == nullptr) return 1;

    SoapySDR::Stream *rx = nullptr;
    SoapySDR::Stream *tx = nullptr;
    int exitCode = 0;
    try {
        device->setFrequency(SOAPY_SDR_RX, 0, 2.38e9);
        device->setBandwidth(SOAPY_SDR_RX, 0, 16e6);
        // TDD exercises ownership/recovery rather than peak RX throughput.
        // Use a clean control-plane rate even on hosts whose USB Ethernet
        // adapter has been degraded by simultaneous high-rate SDR traffic.
        device->setSampleRate(SOAPY_SDR_RX, 0, 4e6);
        device->setSampleRate(SOAPY_SDR_TX, 0, 4e6);
        device->setGain(SOAPY_SDR_TX, 0, 4);

        // A normal transceiver application creates both handles up front and
        // activates only one direction at a time.
        rx = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0});
        tx = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});

        // Activating the other handle while RX is live must fail explicitly;
        // silently switching ownership would surprise ordinary Soapy clients.
        if (device->activateStream(rx) != 0)
            throw std::runtime_error("collision-test RX activation failed");
        const int collision = device->activateStream(tx);
        if (collision != SOAPY_SDR_STREAM_ERROR)
            throw std::runtime_error("simultaneous TX activation was not rejected");
        if (device->deactivateStream(rx) != 0)
            throw std::runtime_error("collision-test RX deactivation failed");

        if (count > device->getStreamMTU(tx))
            throw std::runtime_error("TX sample count exceeds stream MTU");
        constexpr double pi = 3.14159265358979323846;
        std::vector<int16_t> samples(count * 2);
        for (std::size_t i = 0; i < count; ++i) {
            samples[2 * i] = static_cast<int16_t>(
                std::lround(20000.0 * std::cos(2.0 * pi * i / 32.0)));
            samples[2 * i + 1] = static_cast<int16_t>(
                std::lround(20000.0 * std::sin(2.0 * pi * i / 32.0)));
        }
        const void *buffers[] = {samples.data()};
        std::size_t received = 0;
        std::size_t writtenTotal = 0;
        for (unsigned cycle = 0; cycle < cycles; ++cycle) {
            received += receiveSome(device, rx);
            if (device->activateStream(tx) != 0)
                throw std::runtime_error("TX activateStream failed");
            // Alternate explicit burst boundaries with an unflagged final
            // fragment. The latter must be flushed by deactivateStream()
            // before RX is reactivated.
            int flags = (cycle & 1u) == 0u ? SOAPY_SDR_END_BURST : 0;
            const int written = device->writeStream(
                tx, buffers, count, flags, 0, 2'000'000);
            if (written != static_cast<int>(count))
                throw std::runtime_error(std::string("TX writeStream failed: ") +
                                         SoapySDR::errToStr(written));
            writtenTotal += static_cast<std::size_t>(written);
            if (device->deactivateStream(tx) != 0)
                throw std::runtime_error("TX deactivateStream failed");
            received += receiveSome(device, rx);
        }
        std::cout << "SoapySDR repeated half-duplex RX->TX->RX: cycles="
                  << cycles << " rx=" << received << " tx=" << writtenTotal
                  << " simultaneous_activation=rejected"
                  << " deactivation_flush=OK\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        exitCode = 1;
    }
    for (SoapySDR::Stream *stream : {tx, rx}) {
        if (stream == nullptr) continue;
        try { device->deactivateStream(stream); } catch (...) {}
        try { device->closeStream(stream); } catch (...) {}
    }
    SoapySDR::Device::unmake(device);
    return exitCode;
}
