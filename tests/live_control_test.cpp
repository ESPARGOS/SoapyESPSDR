#include <SoapySDR/Device.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "usage: espsdr_live_control_test HOST_OR_DEVICE_ARGS\n";
        return 2;
    }
    const std::string selector = argv[1];
    SoapySDR::Device *device = SoapySDR::Device::make(
        "driver=espsdr," +
        (selector.find('=') == std::string::npos ? "host=" + selector : selector));
    if (device == nullptr) return 1;
    const double oldFrequency = device->getFrequency(SOAPY_SDR_RX, 0);
    const double oldCorrection = device->getFrequencyCorrection(SOAPY_SDR_RX, 0);
    const double oldBandwidth = device->getBandwidth(SOAPY_SDR_RX, 0);
    const double oldGain = device->getGain(SOAPY_SDR_RX, 0);
    const bool oldAgc = device->getGainMode(SOAPY_SDR_RX, 0);
    const std::string oldCycleTotal = device->readSetting("cycle_total");
    const std::string oldCycleStream = device->readSetting("cycle_stream");
    int result = 0;
    try {
        std::cout << "set frequency\n";
        device->setFrequency(SOAPY_SDR_RX, 0, 2437125000.0);
        std::cout << "set frequency correction\n";
        if (!device->hasFrequencyCorrection(SOAPY_SDR_RX, 0))
            throw std::runtime_error("frequency correction is not advertised");
        device->setFrequencyCorrection(SOAPY_SDR_RX, 0, 8.272);
        std::cout << "set bandwidth\n";
        device->setBandwidth(SOAPY_SDR_RX, 0, 21e6);
        std::cout << "set gain\n";
        device->setGain(SOAPY_SDR_RX, 0, 40);
        if (device->getFrequency(SOAPY_SDR_RX, 0) != 2437125000.0) throw std::runtime_error("frequency readback mismatch");
        if (std::abs(device->getFrequencyCorrection(SOAPY_SDR_RX, 0) - 8.272) > 0.0005)
            throw std::runtime_error("frequency-correction readback mismatch");
        if (device->getBandwidth(SOAPY_SDR_RX, 0) != 21e6) throw std::runtime_error("bandwidth readback mismatch");
        if (device->getGain(SOAPY_SDR_RX, 0) != 40) throw std::runtime_error("gain readback mismatch");
        std::cout << "set duty cycle\n";
        device->writeSetting("cycle_total", "5");
        device->writeSetting("cycle_stream", "2");
        if (device->readSetting("cycle_total") != "5" ||
            device->readSetting("cycle_stream") != "2") throw std::runtime_error("duty-cycle readback mismatch");
        const std::vector<double> rates = device->listSampleRates(SOAPY_SDR_RX, 0);
        if (rates.size() != 1 || rates.front() != 2e6)
            throw std::runtime_error("sample-rate list mismatch");
        std::cout << "frequency, correction, analog bandwidth, gain, duty cycle, and sample rates: OK\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    try {
        device->setFrequency(SOAPY_SDR_RX, 0, oldFrequency);
        device->setFrequencyCorrection(SOAPY_SDR_RX, 0, oldCorrection);
        device->setBandwidth(SOAPY_SDR_RX, 0, oldBandwidth);
        device->setGain(SOAPY_SDR_RX, 0, oldGain);
        device->setGainMode(SOAPY_SDR_RX, 0, oldAgc);
        device->writeSetting("cycle_total", oldCycleTotal);
        device->writeSetting("cycle_stream", oldCycleStream);
    } catch (const std::exception &error) {
        std::cerr << "restore failed: " << error.what() << '\n';
        result = 1;
    }
    SoapySDR::Device::unmake(device);
    return result;
}
