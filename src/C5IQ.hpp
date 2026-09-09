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

// IQ8 retains the upper eight bits of each signed IQ10 field; no rounding.
inline std::vector<uint8_t> c5PackIQ8(const std::vector<uint32_t> &words) {
    std::vector<uint8_t> bytes(words.size()*2);
    for(size_t j=0;j<words.size();j++){bytes[2*j]=(words[j]>>2)&255;bytes[2*j+1]=(words[j]>>12)&255;}
    return bytes;
}
inline std::vector<uint32_t> c5UnpackIQ8(const std::vector<uint8_t> &bytes,size_t count) {
    if(bytes.size()!=count*2)throw std::runtime_error("C5: incorrect IQ8 size");
    std::vector<uint32_t> words(count);
    for(size_t j=0;j<count;j++)words[j]=(uint32_t(bytes[2*j])<<2)|(uint32_t(bytes[2*j+1])<<12);
    return words;
}
