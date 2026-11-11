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

UDPRadio::UDPRadio(bool logToFile, bool logToSocket, uint8_t channel,
    const std::filesystem::path & logRoot)
: IRadio(logToFile, logToSocket, channel, logRoot, "UDP")
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
      auto& udpLink = keyValue.second;
      if (udpLink->is_connected())
      {
          udpLink->send(packet, responsePacket, true);
          const Transfer transfer{++m_transferIndex, std::chrono::system_clock::now()};
          const libcrtp::CrtpLinkIdentifier destination{
              keyValue.first.first, keyValue.first.second, link->datarate, true};
          m_logger.log(&destination, packet, responsePacket, false, transfer);
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
    const Transfer transfer{++m_transferIndex, std::chrono::system_clock::now()};
    m_logger.log(link, packet, responsePacket, !libcrtp::isNullPacket(responsePacket), transfer);
    if (!m_links[key]->is_connected()) return false; 
  }
  return true;
}


} // namespace libradio::udpradio
