#include <SoapySDR/Device.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "usage: espsdr_live_control_test HOST\n";
        return 2;
    }
    SoapySDR::Device *device = SoapySDR::Device::make("driver=espsdr,host=" + std::string(argv[1]));
    if (device == nullptr) return 1;
    const double oldFrequency = device->getFrequency(SOAPY_SDR_RX, 0);
    const double oldBandwidth = device->getBandwidth(SOAPY_SDR_RX, 0);
    const double oldGain = device->getGain(SOAPY_SDR_RX, 0);
    const bool oldAgc = device->getGainMode(SOAPY_SDR_RX, 0);
    int result = 0;
    try {
        std::cout << "set frequency\n";
        device->setFrequency(SOAPY_SDR_RX, 0, 2437125000.0);
        std::cout << "set bandwidth\n";
        device->setBandwidth(SOAPY_SDR_RX, 0, 21e6);
        std::cout << "set gain\n";
        device->setGain(SOAPY_SDR_RX, 0, 40);
        if (device->getFrequency(SOAPY_SDR_RX, 0) != 2437125000.0) throw std::runtime_error("frequency readback mismatch");
        if (device->getBandwidth(SOAPY_SDR_RX, 0) != 21e6) throw std::runtime_error("bandwidth readback mismatch");
        if (device->getGain(SOAPY_SDR_RX, 0) != 40) throw std::runtime_error("gain readback mismatch");
        std::cout << "frequency, analog bandwidth, and gain controls: OK\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    try {
        device->setFrequency(SOAPY_SDR_RX, 0, oldFrequency);
        device->setBandwidth(SOAPY_SDR_RX, 0, oldBandwidth);
        device->setGain(SOAPY_SDR_RX, 0, oldGain);
        device->setGainMode(SOAPY_SDR_RX, 0, oldAgc);
    } catch (const std::exception &error) {
        std::cerr << "restore failed: " << error.what() << '\n';
        result = 1;
    }
    SoapySDR::Device::unmake(device);
    return result;
}
