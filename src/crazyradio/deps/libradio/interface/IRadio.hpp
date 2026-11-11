#pragma once

#include <optional>

#include "interface/crtp_logger.hpp"
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

    /** Average received signal strength in dBm; unavailable for unsupported transports. */
    virtual std::optional<double> getLinkRssi(
        const libcrtp::CrtpLinkIdentifier*) const { return std::nullopt; }
protected:
    IRadio(bool logToFile, bool logToSocket, uint8_t channel,
        const std::filesystem::path & logRoot, const std::string & transport)
    : m_logger(logToFile, logToSocket, channel, transport, logRoot) {}
    CrtpLogger m_logger;

};

} // namespace libradio
