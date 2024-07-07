#pragma once

#include <stdint.h>

namespace libcrtp {

    enum CrtpPort {
        CONSOLE             = 0,
        PARAMETERS          = 2,
        COMMANDER           = 3,
        MEMORY_ACCESS       = 4,
        DATA_LOGGING        = 5,
        LOCALIZATION        = 6,
        GENERIC_SETPOINT    = 7,
        PLATFORM            = 13,
        CLIENT_SIDE_DEBUG   = 14,
        LINK_LAYER          = 15,
        NO_PORT             = 0xFF
    };

    struct CrtpPacket
    {
        CrtpPort port;
        uint8_t channel;
        uint8_t data[31];
        uint8_t dataLength;
    
        bool expectsResponse;
        uint8_t matchingBytes;
        bool obeysOrdering;
    };
}; // namespace libcrtp