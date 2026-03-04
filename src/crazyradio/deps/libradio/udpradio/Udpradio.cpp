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

// ---- Tuneables (match socketlink behavior reasonably) ----
constexpr int kMaxPayload = 31;                 // CRTP max data size (excluding header)
constexpr int kMaxDatagram = 1 + kMaxPayload;   // header + payload
constexpr uint8_t kHandshakeByte = 0xF3;

constexpr int kHandshakeRetries = 10;          // overall attempts (socketlink loops forever)
constexpr int kHandshakeReplyTimeoutMs = 10;    // socketlink delays 10ms between tries
constexpr int kHandshakeBurstCount = 10;        // socketlink tries up to 10 receives per send

constexpr int kRxTimeoutMs = 100;               // similar to socketlinkReceiveCRTPPacket 100ms

// Wait for fd readable, returns true if readable before timeout
bool waitReadable(int fd, int timeoutMs) {
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  timeval tv;
  tv.tv_sec = timeoutMs / 1000;
  tv.tv_usec = (timeoutMs % 1000) * 1000;

  int rc = select(fd + 1, &rfds, nullptr, nullptr, &tv);
  return rc > 0 && FD_ISSET(fd, &rfds);
}

uint8_t getHeader(const libcrtp::CrtpPacket& p) {
  return p.port << 4 | p.channel;
}

const uint8_t* getPayloadPtr(const libcrtp::CrtpPacket& p) {
  return p.data; 
}

uint8_t getPayloadSize(const libcrtp::CrtpPacket& p) {
  return p.dataLength;
}

void setFromWire(libcrtp::CrtpPacket& out, const uint8_t* buf, size_t len) {
  if (len < 1) {
    memcpy(&out, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
    std::cerr << "UdpRadio: received empty packet\n";
    return;
  }
  out.port = (libcrtp::CrtpPort)(buf[0] >> 4 & 0xF);
  out.channel = buf[0] & 0b11;
  for (int i = 0; i  < len -1; ++i) {
    out.data[i] = buf[1 + i];
  }
  out.dataLength = len - 1;
}


UDPRadio::UDPRadio() 
{
    //auto fm = firmwarelink::Firmwarelink(19950);

    //uint8_t buf[] = {kHandshakeByte};
    //fm.send(buf, 1);
}

UDPRadio::~UDPRadio()
{   
    std::cerr << "UDP Radio stopped." << std::endl;
}

bool UDPRadio::sendCrtpPacket(
    const libcrtp::CrtpLinkIdentifier * link,
    const libcrtp::CrtpPacket * packet,
    libcrtp::CrtpPacket * responsePacket)
{   
    auto key = std::make_pair(link->channel, link->address);
    auto it = m_links.find(key);
    if (it == m_links.end()) {
        std::cerr << "UdpRadio: LinkIdentifier not found\n";

        m_links[key] = std::make_unique<firmwarelink::Firmwarelink>(19950);
    }

    uint8_t buf[kMaxDatagram];
    buf[0] = getHeader(*packet);
    memcpy(&buf[1], getPayloadPtr(*packet), getPayloadSize(*packet));

    if (getHeader(*packet) != 0xF3)
    {
        //std::cerr << "UdpRadio: Sending packet with header " << std::hex << (int)buf[0] << std::dec << " and payload size " << (int)getPayloadSize(*packet) << "\n";
        // for (int i = 0; i < getPayloadSize(*packet); ++i) {
        //     std::cerr << std::hex << (int)buf[1 + i] << " ";
        // }
        // std::cerr << std::dec << "\n";
    }

    uint8_t receive_buf[kMaxDatagram];
    int size = m_links[key]->send(buf, 1 + getPayloadSize(*packet), receive_buf);

    if (size > 0)
    {
        responsePacket->channel = receive_buf[0] & 0b11;
        responsePacket->port = (libcrtp::CrtpPort)(receive_buf[0] >> 4 & 0xF);
        memcpy(responsePacket->data, &receive_buf[1], size - 1);
        responsePacket->dataLength = size - 1;
    } else {
        memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
    }
    
    return true;
}


} // namespace libradio::udpradio