#pragma once
#include <cstdint>
#include <stdexcept>
#include <vector>

// Lossless little-endian bitstream: two packed I10/Q10 words per five bytes.
inline std::vector<uint8_t> c5PackIQ(const std::vector<uint32_t> &words) {
    std::vector<uint8_t> bytes;
    bytes.reserve((words.size()*20+7)/8);
    uint64_t accumulator=0;unsigned bits=0;
    for(uint32_t word:words) {
        accumulator|=uint64_t(word&0xfffff)<<bits;bits+=20;
        while(bits>=8){bytes.push_back(accumulator&255);accumulator>>=8;bits-=8;}
    }
    if(bits)bytes.push_back(accumulator&255);
    return bytes;
}
inline std::vector<uint32_t> c5UnpackIQ(const std::vector<uint8_t> &bytes,size_t count) {
    if(bytes.size()!=(count*20+7)/8)throw std::runtime_error("C5: incorrect packed IQ size");
    std::vector<uint32_t> words;words.reserve(count);
    uint64_t accumulator=0;unsigned bits=0;size_t offset=0;
    for(size_t j=0;j<count;j++) {
        while(bits<20){accumulator|=uint64_t(bytes[offset++])<<bits;bits+=8;}
        words.push_back(accumulator&0xfffff);accumulator>>=20;bits-=20;
    }
    return words;
}
