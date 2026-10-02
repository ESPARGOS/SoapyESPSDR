#include "Protocol.hpp"
#include <iostream>
#include <vector>
static void require(bool ok) {
    if (!ok)
        throw std::runtime_error("Protocol check failed");
}
int main() {
    std::vector<uint8_t> b(espsdr::packetBytes);
    std::memcpy(b.data(), "ESR2", 4);
    b[4] = 2;
    b[6] = 8;
    b[8] = 7;
    uint32_t rate = 16000000;
    for (unsigned i = 0; i < 4; i++)
        b[12 + i] = (rate >> (8 * i)) & 255;
    b[20] = 1;
    b[24] = 123;
    b[32] = 128;
    b[33] = 127;
    auto p = espsdr::decode(b.data(), b.size());
    require(p.epoch == 7 && p.sample == (1ull << 32) && p.timeUs == 123 && p.iq[0] == -128 &&
            p.iq[1] == 127);
    for (uint32_t supported : {4000000u, 8000000u, 16000000u, 20000000u, 40000000u}) {
        auto copy = b;
        for (unsigned i = 0; i < 4; i++)
            copy[12 + i] = (supported >> (8 * i)) & 255;
        require(espsdr::decode(copy.data(), copy.size()).rate == supported);
    }
    for (auto index : {0, 4, 6, 12}) {
        auto copy = b;
        copy[index] ^= 0xff;
        bool failed = false;
        try {
            espsdr::decode(copy.data(), copy.size());
        } catch (...) {
            failed = true;
        }
        require(failed);
    }
    for (auto length : {0u, 31u, 1375u}) {
        bool failed = false;
        try {
            espsdr::decode(b.data(), length);
        } catch (...) {
            failed = true;
        }
        require(failed);
    }
    // USB URBs can split any byte, including the header, and a canceled
    // previous acquisition can leave a prefix from the middle of a packet.
    for (size_t split = 0; split <= b.size(); ++split) {
        espsdr::PacketAssembler assembler(true);
        std::vector<uint8_t> prefix(53, 0xa5);
        assembler.append(prefix.data(), prefix.size());
        assembler.append(b.data(), split);
        espsdr::Packet packet{};
        bool first = assembler.next(packet);
        require(first == (split == b.size()));
        assembler.append(b.data() + split, b.size() - split);
        require(first || assembler.next(packet));
        require(packet.epoch == 7 && packet.sample == (1ull << 32));
        require(!assembler.next(packet));
        assembler.append(b.data(), b.size());
        require(assembler.next(packet));
        require(!assembler.next(packet));
        auto bad = b;
        bad[0] = 0;
        assembler.append(bad.data(), bad.size());
        bool failed = false;
        try {
            assembler.next(packet);
        } catch (...) {
            failed = true;
        }
        require(failed);
    }
    std::cout << "Wire validation, byte order, signed extrema and 64-bit sample count passed\n";
}
