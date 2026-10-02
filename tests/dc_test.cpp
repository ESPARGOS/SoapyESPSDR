#include "DcRemoval.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
static void require(bool ok, const char *why) {
    if (!ok) {
        std::cerr << why << '\n';
        std::exit(1);
    }
}
int main() {
    constexpr size_t n = 672;
    std::array<int8_t, 2 * n> in;
    std::array<float, 2 * n> out;
    for (unsigned rate : {4000000u, 20000000u, 40000000u}) {
        espsdr::DcRemoval dc;
        dc.reset(rate);
        for (size_t j = 0; j < n; j++) {
            in[2 * j] = 80;
            in[2 * j + 1] = -65;
        }
        dc.process(in.data(), out.data(), n);
        for (auto x : out)
            require(x == 0, "constant DC not removed at startup");
        double energyIn = 0, energyOut = 0, meanI = 0, meanQ = 0;
        size_t count = 0;
        // DC step plus a 100 kHz tone; check settled offset and signal energy.
        for (size_t base = 0; base < rate / 5; base += n) {
            for (size_t j = 0; j < n; j++) {
                double phase = 2 * 3.141592653589793 * 100000 * (base + j) / rate;
                in[2 * j] = int8_t(std::round(30 + 40 * std::cos(phase)));
                in[2 * j + 1] = int8_t(std::round(-20 + 40 * std::sin(phase)));
            }
            dc.process(in.data(), out.data(), n);
            if (base > rate / 10)
                for (size_t j = 0; j < n; j++) {
                    meanI += out[2 * j];
                    meanQ += out[2 * j + 1];
                    count++;
                    energyOut += out[2 * j] * out[2 * j] + out[2 * j + 1] * out[2 * j + 1];
                    energyIn += std::pow(in[2 * j] - 30, 2) + std::pow(in[2 * j + 1] + 20, 2);
                }
        }
        require(std::hypot(meanI / count, meanQ / count) < .03, "DC step failed to settle");
        require(std::abs(10 * std::log10(energyOut / energyIn)) < .01, "tone power changed");
    }
    std::cout << "DC startup, drift, and tone preservation passed at 4/20/40 MSa/s\n";
}
