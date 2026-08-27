#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> running{true};

void stopHandler(int)
{
    running = false;
}

struct Options {
    std::string host = "esp-sdr.local";
    std::string deviceArgs;
    double rate = 2e6;
    double seconds = 10.0;
    double interval = 1.0;
    unsigned cycleTotal = 1;
    unsigned cycleStream = 1;
};

void usage(const char *program)
{
    std::cerr << "Usage: " << program
              << " [--host HOST | --device ARGS] [--rate HZ] [--seconds N] [--interval N]"
                 " [--cycle-total N] [--cycle-stream N]\n";
}

Options parseOptions(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--help" || argument == "-h") {
            usage(argv[0]);
            std::exit(0);
        }
        if (i + 1 >= argc) throw std::runtime_error("missing value after " + argument);
        const std::string value = argv[++i];
        if (argument == "--host") options.host = value;
        else if (argument == "--device") options.deviceArgs = value;
        else if (argument == "--rate") options.rate = std::stod(value);
        else if (argument == "--seconds") options.seconds = std::stod(value);
        else if (argument == "--interval") options.interval = std::stod(value);
        else if (argument == "--cycle-total") options.cycleTotal = std::stoul(value);
        else if (argument == "--cycle-stream") options.cycleStream = std::stoul(value);
        else throw std::runtime_error("unknown option: " + argument);
    }
    if (options.rate <= 0 || options.seconds <= 0 || options.interval <= 0) {
        throw std::runtime_error("rate, seconds, and interval must be positive");
    }
    if (options.cycleTotal == 0 || options.cycleStream == 0 ||
        options.cycleStream > options.cycleTotal) {
        throw std::runtime_error("duty cycle requires 1 <= cycle-stream <= cycle-total");
    }
    return options;
}

uint64_t sensor(SoapySDR::Device *device, const char *name)
{
    return std::stoull(device->readSensor(name));
}

struct Counters {
    uint64_t frames = 0;
    uint64_t lost = 0;
    uint64_t firmwareDrops = 0;
    uint64_t invalidDatagrams = 0;
    uint64_t duplicateDatagrams = 0;
    uint64_t reorderedDatagrams = 0;
    uint64_t datagramGaps = 0;
    uint64_t lateDatagramsRecovered = 0;
    uint64_t queueDrops = 0;
    uint64_t datagrams = 0;
    uint64_t overflows = 0;
    uint64_t timeouts = 0;
    uint64_t samples = 0;
};

Counters snapshot(SoapySDR::Device *device, uint64_t overflows,
                  uint64_t timeouts, uint64_t samples)
{
    Counters counters;
    counters.frames = sensor(device, "completed_frames");
    counters.lost = sensor(device, "lost_chunks");
    counters.firmwareDrops = sensor(device, "firmware_drops");
    counters.invalidDatagrams = sensor(device, "invalid_datagrams");
    counters.duplicateDatagrams = sensor(device, "duplicate_datagrams");
    counters.reorderedDatagrams = sensor(device, "reordered_datagrams");
    counters.datagramGaps = sensor(device, "datagram_gaps");
    counters.lateDatagramsRecovered = sensor(device, "late_datagrams_recovered");
    counters.queueDrops = sensor(device, "queue_drops");
    counters.datagrams = sensor(device, "datagrams");
    counters.overflows = overflows;
    counters.timeouts = timeouts;
    counters.samples = samples;
    return counters;
}

Counters difference(const Counters &now, const Counters &before)
{
    return {
        now.frames - before.frames,
        now.lost - before.lost,
        now.firmwareDrops - before.firmwareDrops,
        now.invalidDatagrams - before.invalidDatagrams,
        now.duplicateDatagrams - before.duplicateDatagrams,
        now.reorderedDatagrams - before.reorderedDatagrams,
        now.datagramGaps - before.datagramGaps,
        now.lateDatagramsRecovered - before.lateDatagramsRecovered,
        now.queueDrops - before.queueDrops,
        now.datagrams - before.datagrams,
        now.overflows - before.overflows,
        now.timeouts - before.timeouts,
        now.samples - before.samples,
    };
}

double lossPercent(const Counters &counters)
{
    const uint64_t total = counters.frames + counters.lost;
    return total == 0 ? 0.0 : 100.0 * counters.lost / total;
}

void printReport(double elapsed, double period, const Counters &delta,
                 const Counters &total)
{
    std::cout << std::fixed << std::setprecision(3)
              << elapsed << ' '
              << delta.samples / period / 1e6 << ' '
              << delta.frames / period << ' '
              << delta.lost / period << ' '
              << lossPercent(delta) << ' '
              << lossPercent(total) << ' '
              << delta.overflows / period << ' '
              << delta.firmwareDrops / period << ' '
              << delta.invalidDatagrams / period << ' '
              << delta.duplicateDatagrams / period << ' '
              << delta.reorderedDatagrams / period << ' '
              << delta.datagramGaps / period << ' '
              << delta.lateDatagramsRecovered / period << ' '
              << delta.queueDrops / period << ' '
              << delta.datagrams / period << '\n';
}

} // namespace

int main(int argc, char **argv)
{
    Options options;
    try {
        options = parseOptions(argc, argv);
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        usage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, stopHandler);
    std::signal(SIGTERM, stopHandler);

    SoapySDR::Device *device = nullptr;
    SoapySDR::Stream *stream = nullptr;
    try {
        const std::string selector = options.deviceArgs.empty()
            ? "host=" + options.host : options.deviceArgs;
        device = SoapySDR::Device::make(
            "driver=espsdr," + selector +
            ",cycle_total=" + std::to_string(options.cycleTotal) +
            ",cycle_stream=" + std::to_string(options.cycleStream));
        if (device == nullptr) throw std::runtime_error("failed to open SoapyESPSDR device");
        const double oldRate = device->getSampleRate(SOAPY_SDR_RX, 0);
        device->setSampleRate(SOAPY_SDR_RX, 0, options.rate);
        stream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
        if (device->activateStream(stream) != 0) throw std::runtime_error("activateStream failed");

        std::vector<int16_t> buffer(16384 * 2);
        void *buffers[] = {buffer.data()};
        uint64_t receivedSamples = 0;
        uint64_t overflowEvents = 0;
        uint64_t timeoutEvents = 0;
        const auto start = std::chrono::steady_clock::now();
        auto previousTime = start;
        auto nextReport = start + std::chrono::duration<double>(options.interval);
        Counters previous = snapshot(device, 0, 0, 0);

        std::cout << "# time_s rx_MSps received_IQ_frames_s missing_IQ_frames_s "
                     "interval_loss_pct total_loss_pct overflows_s firmware_drops_s "
                     "invalid_datagrams_s duplicate_datagrams_s reordered_datagrams_s "
                     "datagram_gaps_s late_datagrams_recovered_s queue_drops_s datagrams_s\n";

        while (running) {
            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - start).count() >= options.seconds) break;
            int flags = 0;
            long long timeNs = 0;
            const int count = device->readStream(
                stream, buffers, 16384, flags, timeNs, 250000);
            if (count > 0) receivedSamples += static_cast<unsigned>(count);
            else if (count == SOAPY_SDR_OVERFLOW) ++overflowEvents;
            else if (count == SOAPY_SDR_TIMEOUT) ++timeoutEvents;
            else throw std::runtime_error("readStream failed with " + std::to_string(count));

            const auto afterRead = std::chrono::steady_clock::now();
            if (afterRead >= nextReport) {
                const Counters current = snapshot(
                    device, overflowEvents, timeoutEvents, receivedSamples);
                const Counters delta = difference(current, previous);
                const double period = std::chrono::duration<double>(afterRead - previousTime).count();
                printReport(
                    std::chrono::duration<double>(afterRead - start).count(),
                    period, delta, current);
                previous = current;
                previousTime = afterRead;
                nextReport = afterRead + std::chrono::duration<double>(options.interval);
            }
        }

        const auto end = std::chrono::steady_clock::now();
        const Counters total = snapshot(device, overflowEvents, timeoutEvents, receivedSamples);
        std::cout << "# summary elapsed_s=" << std::fixed << std::setprecision(3)
                  << std::chrono::duration<double>(end - start).count()
                  << " samples=" << total.samples
                  << " received_IQ_frames=" << total.frames
                  << " missing_IQ_frames=" << total.lost
                  << " loss_pct=" << lossPercent(total)
                  << " overflow_events=" << total.overflows
                  << " firmware_drops=" << total.firmwareDrops
                  << " invalid_datagrams=" << total.invalidDatagrams
                  << " duplicate_datagrams=" << total.duplicateDatagrams
                  << " reordered_datagrams=" << total.reorderedDatagrams
                  << " datagram_gaps=" << total.datagramGaps
                  << " late_datagrams_recovered=" << total.lateDatagramsRecovered
                  << " queue_drops=" << total.queueDrops
                  << " timeouts=" << total.timeouts << '\n';

        device->deactivateStream(stream);
        device->closeStream(stream);
        stream = nullptr;
        if (std::abs(oldRate - 2e6) < 1) {
            device->setSampleRate(SOAPY_SDR_RX, 0, oldRate);
        }
        SoapySDR::Device::unmake(device);
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        if (device != nullptr && stream != nullptr) {
            try { device->deactivateStream(stream); } catch (...) {}
            try { device->closeStream(stream); } catch (...) {}
        }
        if (device != nullptr) SoapySDR::Device::unmake(device);
        return 1;
    }
}
