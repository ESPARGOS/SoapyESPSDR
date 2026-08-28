#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>

#include <algorithm>
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
    const double oldSampleRate = device->getSampleRate(SOAPY_SDR_RX, 0);
    const double oldTxSampleRate = device->getSampleRate(SOAPY_SDR_TX, 0);
    const double oldGain = device->getGain(SOAPY_SDR_RX, 0);
    const bool oldAgc = device->getGainMode(SOAPY_SDR_RX, 0);
    const std::string oldFilterOverride = device->readSetting("rx_filter_override");
    const std::string oldTxGainCode = device->readSetting("tx_gain_code");
    const std::string oldTxWireFormat = device->readSetting("tx_wire_format");
    const std::string oldCycleTotal = device->readSetting("cycle_total");
    const std::string oldCycleStream = device->readSetting("cycle_stream");
    SoapySDR::Stream *txStream = nullptr;
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
        if (rates.size() != 10 || rates.front() != 16e6 ||
            std::find(rates.begin(), rates.end(), 8e6) == rates.end() ||
            std::find(rates.begin(), rates.end(), 2e6) == rates.end())
            throw std::runtime_error("sample-rate list mismatch");
        for (const double rate : rates) {
            std::cout << "set sample rate " << rate << '\n';
            device->setSampleRate(SOAPY_SDR_RX, 0, rate);
            if (std::abs(device->getSampleRate(SOAPY_SDR_RX, 0) - rate) > 1.0)
                throw std::runtime_error("sample-rate readback mismatch");
        }
        const bool usbTransport = device->getHardwareInfo().at(
            "transport").find("USB") != std::string::npos;
        const std::vector<double> txRates =
            device->listSampleRates(SOAPY_SDR_TX, 0);
        const double usbCeiling = 320e6 / 60.0;
        const bool hasUsbCeiling = std::find_if(
            txRates.begin(), txRates.end(), [usbCeiling](double rate) {
                return std::abs(rate - usbCeiling) < 1.0;
            }) != txRates.end();
        if (hasUsbCeiling != usbTransport)
            throw std::runtime_error("transport-specific TX ceiling mismatch");
        for (const double rate : txRates) {
            device->setSampleRate(SOAPY_SDR_TX, 0, rate);
            if (std::abs(device->getSampleRate(SOAPY_SDR_TX, 0) - rate) > 1.0)
                throw std::runtime_error("TX sample-rate readback mismatch");
        }
        std::cout << "set calibrated TX gain\n";
        const SoapySDR::Range txGainRange =
            device->getGainRange(SOAPY_SDR_TX, 0);
        if (std::abs(txGainRange.minimum() - 0.0) > 1e-9 ||
            std::abs(txGainRange.maximum() - 19.57) > 1e-9)
            throw std::runtime_error("TX gain range mismatch");
        device->setGain(SOAPY_SDR_TX, 0, 3.44);
        if (std::abs(device->getGain(SOAPY_SDR_TX, 0) - 3.44) > 1e-9 ||
            device->readSetting("tx_gain_code") != "2")
            throw std::runtime_error("calibrated TX gain mapping mismatch");
        device->setGain(SOAPY_SDR_TX, 0, 100.0);
        if (std::abs(device->getGain(SOAPY_SDR_TX, 0) - 19.57) > 1e-9 ||
            device->readSetting("tx_gain_code") != "22")
            throw std::runtime_error("calibrated TX gain clamp mismatch");
        device->writeSetting("tx_gain_code", "8");
        if (device->readSetting("tx_gain_code") != "8" ||
            !std::isnan(device->getGain(SOAPY_SDR_TX, 0)))
            throw std::runtime_error("expert TX gain code mismatch");
        std::cout << "check TX wire-format policy\n";
        const auto settingInfo = device->getSettingInfo();
        const auto wireInfo = std::find_if(
            settingInfo.begin(), settingInfo.end(), [](const auto &info) {
                return info.key == "tx_wire_format";
            });
        if (wireInfo == settingInfo.end() ||
            wireInfo->options != std::vector<std::string>(
                {"auto", "iq8", "iq10"}))
            throw std::runtime_error("TX wire-format options mismatch");
        if (usbTransport) {
            bool usbOverrideRejected = false;
            try {
                device->writeSetting("tx_wire_format", "iq8");
            } catch (const std::runtime_error &) {
                usbOverrideRejected = true;
            }
            if (!usbOverrideRejected)
                throw std::runtime_error(
                    "USB accepted an Ethernet wire-format override");
        } else {
            for (const char *format : {"iq8", "iq10"}) {
                device->writeSetting("tx_wire_format", format);
                if (device->readSetting("tx_wire_format") != format)
                    throw std::runtime_error(
                        "TX wire-format forced readback mismatch");
            }
        }
        device->writeSetting("tx_wire_format", "auto");
        if (device->readSetting("tx_wire_format") != "auto")
            throw std::runtime_error("TX wire-format readback mismatch");
        txStream = device->setupStream(
            SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
        if (device->activateStream(txStream) != 0)
            throw std::runtime_error("TX activation failed");
        bool activeChangeRejected = false;
        try {
            device->writeSetting("tx_wire_format", "auto");
        } catch (const std::runtime_error &) {
            activeChangeRejected = true;
        }
        if (!activeChangeRejected)
            throw std::runtime_error(
                "active TX wire-format change was not rejected");
        if (device->deactivateStream(txStream) != 0)
            throw std::runtime_error("TX deactivation failed");
        device->closeStream(txStream);
        txStream = nullptr;
        std::cout << "check typed AGC sensors\n";
        device->setGainMode(SOAPY_SDR_RX, 0, true);
        if (device->readSensor("rx_agc_active") != "true")
            throw std::runtime_error("AGC active sensor mismatch");
        if (device->getSensorInfo("rx_agc_active").type !=
            SoapySDR::ArgInfo::BOOL)
            throw std::runtime_error("AGC active sensor type mismatch");
        if (device->getSensorInfo("rx_agc_current_gain").units != "dB")
            throw std::runtime_error("AGC gain sensor units mismatch");
        (void)std::stoul(device->readSensor("rx_agc_current_gain"));
        (void)std::stoul(device->readSensor("rx_agc_robust_peak"));
        (void)std::stoul(device->readSensor("rx_agc_gain_changes"));
        std::cout << "frequency, correction, analog bandwidth, RX/TX gain, duty cycle, and sample rates: OK\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    if (txStream != nullptr) {
        try {
            device->deactivateStream(txStream);
            device->closeStream(txStream);
        } catch (const std::exception &error) {
            std::cerr << "TX stream cleanup failed: " << error.what() << '\n';
            result = 1;
        }
    }
    try {
        device->setFrequency(SOAPY_SDR_RX, 0, oldFrequency);
        device->setFrequencyCorrection(SOAPY_SDR_RX, 0, oldCorrection);
        device->setBandwidth(SOAPY_SDR_RX, 0, oldBandwidth);
        device->setSampleRate(SOAPY_SDR_RX, 0, oldSampleRate);
        device->setSampleRate(SOAPY_SDR_TX, 0, oldTxSampleRate);
        device->writeSetting("rx_filter_override", oldFilterOverride);
        device->writeSetting("tx_gain_code", oldTxGainCode);
        device->writeSetting("tx_wire_format", oldTxWireFormat);
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
