#include "udpradio/Udpradio.hpp"


#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <ostream>


namespace libradio::udpradio {

UDPRadio::UDPRadio() 
{
}

UDPRadio::~UDPRadio()
{   
    std::cerr << "UDP Radio stopped." << std::endl;
}

bool 
UDPRadio::sendCrtpPacket(
    const libcrtp::CrtpLinkIdentifier * link,
    const libcrtp::CrtpPacket * packet,
    libcrtp::CrtpPacket * responsePacket)
{   
  if (link->isBroadcast)
  {
    for (auto & keyValue : m_links)
    {
      auto& link = keyValue.second;
      if (link->is_connected()) 
      {
          link->send(packet, responsePacket, true);
      }
    }
  } else {
    auto key = std::make_pair(link->channel, link->address);
    auto it = m_links.find(key);
    if (it == m_links.end()) 
    {
        uint8_t id = static_cast<uint8_t>((link->address >> 0) & 0xFF);
        m_links[key] = std::make_unique<sitllink::SITLlink>(19850 + id);
    }

    m_links[key]->send(packet, responsePacket, false);
    if (!m_links[key]->is_connected()) return false; 
  }
  return true;
}


} // namespace libradio::udpradio