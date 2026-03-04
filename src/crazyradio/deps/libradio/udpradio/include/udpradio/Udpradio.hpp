#pragma once

#include <stdint.h>
#include "interface/IRadio.hpp"
#include <string>
#include <netinet/in.h>
#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"

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

private:
    void handshake();
    
    std::string m_remoteIp;
    uint16_t m_remotePort;
    
    int m_fd{-1};
    sockaddr_in m_remoteAddr{};
    bool m_handshakeDone{false};

};

} // namepsace libradio::udpradio