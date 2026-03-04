#pragma once

#include "libcrtp/CrtpLink.hpp"
#include "libcrtp/CrtpPacket.hpp"

namespace libradio {

class IRadio {
public:
    virtual ~IRadio() = default;

    virtual bool sendCrtpPacket(
        const libcrtp::CrtpLinkIdentifier* link,
        const libcrtp::CrtpPacket* packet,
        libcrtp::CrtpPacket* responsePacket
    ) = 0;
};

} // namespace libradio

