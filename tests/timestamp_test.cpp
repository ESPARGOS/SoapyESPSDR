#include "TimestampUnwrapper.hpp"

#include <cstdint>
#include <iostream>

int main()
{
    espsdr::TimestampUnwrapper timestamp;

    timestamp.reset((uint64_t{3} << 32) + 100u);
    auto value = timestamp.unwrap(120u);
    if (!value || *value != (uint64_t{3} << 32) + 120u) return 1;

    timestamp.reset((uint64_t{5} << 32) + 0xfffffff0u);
    value = timestamp.unwrap(0xfffffff5u);
    if (!value || *value != (uint64_t{5} << 32) + 0xfffffff5u) return 1;
    value = timestamp.unwrap(10u);
    if (!value || *value != (uint64_t{6} << 32) + 10u) return 1;
    if (timestamp.unwrap(9u)) return 1;
    value = timestamp.unwrap(11u);
    if (!value || *value != (uint64_t{6} << 32) + 11u) return 1;

    timestamp.reset((uint64_t{8} << 32) + 20u);
    value = timestamp.unwrap(25u);
    if (!value || *value != (uint64_t{8} << 32) + 25u) return 1;

    timestamp.reset(0u);
    value = timestamp.unwrap(42u);
    if (!value || *value != 42u) return 1;

    std::cout << "timestamp rollover tests passed\n";
    return 0;
}

