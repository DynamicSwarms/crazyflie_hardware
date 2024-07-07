#pragma once

#include <stdint.h>

namespace libcrtp {
    struct CrtpPacket
    {
        uint8_t port;
        uint8_t channel;
        uint8_t data[31];
        uint8_t dataLength;
    
        bool expectsResponse;
        uint8_t matchingBytes;
        bool obeysOrdering;
    };
}; // namespace libcrtp