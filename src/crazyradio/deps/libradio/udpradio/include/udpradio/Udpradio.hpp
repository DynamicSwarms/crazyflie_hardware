#pragma once

#include <stdint.h>
#include "interface/IRadio.hpp"
#include <string>
#include <netinet/in.h>
#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"
#include <map>
#include "udpradio/sitllink.hpp"
#include <memory> 
namespace libradio::udpradio {
class UDPRadio : public libradio::IRadio
{
public:
   
public:
    UDPRadio();
    
    virtual ~UDPRadio();

    bool sendCrtpPacket(
        const libcrtp::CrtpLinkIdentifier * link,
        const libcrtp::CrtpPacket * packet,
        libcrtp::CrtpPacket * responsePacket) override;

    void resetLink(const libcrtp::CrtpLinkIdentifier *) override {}

    double getLinkQuality(const libcrtp::CrtpLinkIdentifier *) const override
    {
        return 1.0;
    }

private:
    void handshake();
    
    std::string m_remoteIp;
    uint16_t m_remotePort;
    
    int m_fd{-1};
    sockaddr_in m_remoteAddr{};
    bool m_handshakeDone{false};

    std::map<std::pair<uint8_t, uint64_t>, std::unique_ptr<sitllink::SITLlink>> m_links; 
};

} // namepsace libradio::udpradio
