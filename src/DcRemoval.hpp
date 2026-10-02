#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace espsdr {
// Host-side residual correction. This cannot recover clipped ADC headroom.
// A 10 Hz single-pole DC estimator: rate-independent and continuous across
// packets/partial application reads. Double precision avoids accumulator
// stagnation at the small coefficient required by 40 MSa/s.
class DcRemoval {
    double meanI = 0, meanQ = 0, alpha = 0;
    bool initialized = false;

  public:
    void reset(uint32_t rate) {
        alpha = -std::expm1(-2 * 3.14159265358979323846 * 10.0 / rate);
        initialized = false;
    }
    void process(const int8_t *in, float *out, size_t samples) {
        if (!samples)
            return;
        if (!initialized) {
            int64_t sumI = 0, sumQ = 0;
            for (size_t j = 0; j < samples; ++j) {
                sumI += in[2 * j];
                sumQ += in[2 * j + 1];
            }
            meanI = double(sumI) / samples;
            meanQ = double(sumQ) / samples;
            initialized = true;
        }
        for (size_t j = 0; j < samples; ++j) {
            const double i = in[2 * j] - meanI, q = in[2 * j + 1] - meanQ;
            meanI += alpha * i;
            meanQ += alpha * q;
            out[2 * j] = float(i);
            out[2 * j + 1] = float(q);
        }
    }
};
} // namespace espsdr
