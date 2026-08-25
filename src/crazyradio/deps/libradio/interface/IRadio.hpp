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

    virtual void resetLink(const libcrtp::CrtpLinkIdentifier* link) = 0;

    /** Return transport-observed delivery quality in the range [0, 1]. */
    virtual double getLinkQuality(
        const libcrtp::CrtpLinkIdentifier* link
    ) const = 0;
};

} // namespace libradio
