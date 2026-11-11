#pragma once

#include <chrono>
#include <cstdint>
#include <ostream>
#include <string>

namespace libradio {
struct Transfer {
    uint64_t id;
    std::chrono::system_clock::time_point completed;
};
void writeLogPrefix(std::ostream & out, const Transfer & transfer);
void writeLogHeader(std::ostream & out, const std::string & transport, uint8_t channel);
} // namespace libradio
