#pragma once
#include "Protocol.hpp"
#include <SoapySDR/Types.hpp>
#include <json/json.h>
#include <memory>
#include <vector>
namespace espsdr {
class Transport {
  public:
    virtual ~Transport() = default;
    virtual Json::Value request(Json::Value value) = 0;
    virtual int receive(uint8_t *data, size_t capacity) = 0;
    virtual void prepare(Json::Value &start) = 0;
    virtual void finish() {}
    virtual std::string name() const = 0;
};
std::unique_ptr<Transport> makeTransport(const SoapySDR::Kwargs &args);
SoapySDR::KwargsList discover(const SoapySDR::Kwargs &args);
} // namespace espsdr
