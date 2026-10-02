#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>
namespace espsdr {
constexpr size_t payloadBytes = 1344, packetBytes = 1376, packetSamples = 672;
constexpr uint16_t usbVid = 0x303a, usbPid = 0x4531, usbRevision = 0x0200;
inline uint64_t little(const uint8_t *p, unsigned n) {
    uint64_t v = 0;
    for (unsigned i = 0; i < n; ++i)
        v |= uint64_t(p[i]) << (8 * i);
    return v;
}
struct Packet {
    uint32_t epoch, rate;
    uint64_t sample, timeUs;
    std::array<int8_t, payloadBytes> iq;
};
inline Packet decode(const uint8_t *data, size_t size) {
    if (size != packetBytes || std::memcmp(data, "ESR2", 4) || little(data + 4, 2) != 2 ||
        little(data + 6, 2) != 8)
        throw std::runtime_error("Invalid ESP-SDR RX v2 packet");
    Packet p{};
    p.epoch = little(data + 8, 4);
    p.rate = little(data + 12, 4);
    p.sample = little(data + 16, 8);
    p.timeUs = little(data + 24, 8);
    if (p.rate != 4000000 && p.rate != 8000000 && p.rate != 16000000 && p.rate != 20000000 &&
        p.rate != 40000000)
        throw std::runtime_error("Invalid packet sample rate");
    std::memcpy(p.iq.data(), data + 32, payloadBytes);
    return p;
}

class PacketAssembler {
    std::vector<uint8_t> pending;
    size_t used = 0;
    bool synchronize;

  public:
    explicit PacketAssembler(bool recoverInitialBoundary) : synchronize(recoverInitialBoundary) {}
    void append(const uint8_t *data, size_t size) {
        pending.insert(pending.end(), data, data + size);
    }
    bool next(Packet &packet) {
        while (pending.size() - used >= packetBytes) {
            const auto *data = pending.data() + used;
            if (synchronize && std::memcmp(data, "ESR2", 4)) {
                ++used;
                continue;
            }
            try {
                packet = decode(data, packetBytes);
            } catch (...) {
                if (!synchronize)
                    throw;
                ++used;
                continue;
            }
            synchronize = false;
            used += packetBytes;
            return true;
        }
        pending.erase(pending.begin(), pending.begin() + used);
        used = 0;
        return false;
    }
};
} // namespace espsdr
