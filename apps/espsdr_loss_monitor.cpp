#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <vector>
static volatile std::sig_atomic_t running = 1;
static void stop(int) { running = 0; }
int main(int argc, char **argv) {
    try {
        std::cout << std::unitbuf;
        std::string args = "driver=espsdr";
        double rate = 16e6, seconds = 30;
        for (int i = 1; i < argc; i++) {
            std::string opt = argv[i];
            if (opt == "--help") {
                std::cout << "--device ARGS --rate HZ --seconds N\n";
                return 0;
            }
            if (++i >= argc)
                throw std::runtime_error("Missing option value");
            if (opt == "--device")
                args = argv[i];
            else if (opt == "--host")
                args = "driver=espsdr,host=" + std::string(argv[i]);
            else if (opt == "--rate")
                rate = std::stod(argv[i]);
            else if (opt == "--seconds")
                seconds = std::stod(argv[i]);
            else
                throw std::runtime_error("Unknown option: " + opt);
        }
        if (!std::isfinite(seconds) || seconds <= 0)
            throw std::runtime_error("Duration must be positive");
        std::unique_ptr<SoapySDR::Device, void (*)(SoapySDR::Device *)> device(
            SoapySDR::Device::make(args), SoapySDR::Device::unmake);
        if (!device)
            throw std::runtime_error("Device not found");
        device->setSampleRate(SOAPY_SDR_RX, 0, rate);
        auto stream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS8);
        auto start = std::chrono::steady_clock::now(), last = start;
        device->activateStream(stream);
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        std::vector<int8_t> data(2 * 65536);
        void *buffers[] = {data.data()};
        uint64_t samples = 0, overflows = 0, timeouts = 0;
        double elapsed = 0;
        while (running && elapsed < seconds) {
            int flags = 0;
            long long time = 0;
            int n = device->readStream(stream, buffers, 65536, flags, time, 100000);
            if (n > 0)
                samples += n;
            else if (n == SOAPY_SDR_OVERFLOW)
                overflows++;
            else if (n == SOAPY_SDR_TIMEOUT)
                timeouts++;
            else
                throw std::runtime_error(SoapySDR::errToStr(n));
            auto now = std::chrono::steady_clock::now();
            elapsed = std::chrono::duration<double>(now - start).count();
            if (now - last >= std::chrono::seconds(1)) {
                std::cout << elapsed << " s: " << samples << " samples, " << samples / elapsed / 1e6
                          << " MS/s, missing=" << device->readSensor("rx_missing_samples")
                          << ", overflows=" << overflows << ", timeouts=" << timeouts << '\n';
                last = now;
            }
        }
        uint64_t missing = std::stoull(device->readSensor("rx_missing_samples"));
        uint64_t queueDrops = std::stoull(device->readSensor("rx_queue_drops"));
        uint64_t invalid = std::stoull(device->readSensor("rx_invalid_packets"));
        std::cout << "Total: samples=" << samples << " missing=" << missing
                  << " queue_drops=" << queueDrops << " invalid=" << invalid
                  << " overflows=" << overflows << " timeouts=" << timeouts
                  << " elapsed=" << elapsed << '\n';
        device->deactivateStream(stream);
        device->closeStream(stream);
        return samples && missing == 0 && queueDrops == 0 && invalid == 0 && overflows == 0 &&
                       timeouts == 0
                   ? 0
                   : 1;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
}
