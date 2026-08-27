#pragma once

#include <cstdint>
#include <optional>

namespace espsdr {

class TimestampUnwrapper {
public:
    void reset(const uint64_t referenceUs)
    {
        _last = 0;
        _wrap = referenceUs & ~uint64_t{0xffffffffu};
        _reference = referenceUs;
        _have = false;
    }

    std::optional<uint64_t> unwrap(const uint32_t timestampUs32)
    {
        if (!_have && _reference != 0u) {
            const uint64_t candidate = _wrap + timestampUs32;
            if (candidate + (uint64_t{1} << 31) < _reference) {
                _wrap += uint64_t{1} << 32;
            } else if (candidate > _reference + (uint64_t{1} << 31) &&
                       _wrap >= (uint64_t{1} << 32)) {
                _wrap -= uint64_t{1} << 32;
            }
        }
        const bool wrapped = _have && timestampUs32 < _last &&
            _last - timestampUs32 > 0x80000000u;
        const bool monotonic = !_have || wrapped || timestampUs32 >= _last;
        if (!monotonic) return std::nullopt;
        if (wrapped) _wrap += uint64_t{1} << 32;
        _last = timestampUs32;
        _have = true;
        return _wrap + timestampUs32;
    }

private:
    uint32_t _last = 0;
    uint64_t _wrap = 0;
    uint64_t _reference = 0;
    bool _have = false;
};

} // namespace espsdr

