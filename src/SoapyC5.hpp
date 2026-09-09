#pragma once
#include <SoapySDR/Device.hpp>
SoapySDR::Device *makeC5(const SoapySDR::Kwargs &args);
SoapySDR::KwargsList findC5(const SoapySDR::Kwargs &args);
