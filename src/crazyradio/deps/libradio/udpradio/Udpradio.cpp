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
m_remoteIp = "127.0.0.1";
m_remotePort = 19850;

  m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (m_fd < 0) {
    throw std::runtime_error("UdpRadio: socket() failed");
  }

  // Bind to ephemeral local port like socketlink does (port 0)
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  local.sin_port = htons(m_remotePort);
  if (::bind(m_fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
    ::close(m_fd);
    m_fd = -1;
    throw std::runtime_error("UdpRadio: bind() failed");
  }

  std::memset(&m_remoteAddr, 0, sizeof(m_remoteAddr));
  m_remoteAddr.sin_family = AF_INET;
  m_remoteAddr.sin_port = htons(m_remotePort);

  if (::inet_pton(AF_INET, m_remoteIp.c_str(), &m_remoteAddr.sin_addr) != 1) {
    ::close(m_fd);
    m_fd = -1;
    throw std::runtime_error("UdpRadio: inet_pton() failed (bad IPv4 address?)");
  }

  // Perform handshake expected by socketlinkInit()
  handshake();
}

UDPRadio::~UDPRadio()
{   
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
    std::cerr << "UDP Radio stopped." << std::endl;
}


  
  void UDPRadio::handshake() {
    uint8_t b = kHandshakeByte;
//std::cerr << "UDP Radio: Listening for any data for 5 seconds...\n";
//
//for (int i = 0; i < 50; ++i) { // 50 iterations of 100ms each
//    if (!waitReadable(m_fd, 100)) {
//        continue;
//    }
//
//    std::array<uint8_t, kMaxDatagram> rx{};
//    sockaddr_in from{};
//    socklen_t fromLen = sizeof(from);
//    ssize_t r = ::recvfrom(
//        m_fd, rx.data(), rx.size(), 0,
//        reinterpret_cast<sockaddr*>(&from), &fromLen
//    );
//
//    if (r > 0) {
//        std::cerr << "UDP Radio: Received " << r << " bytes: ";
//        for (ssize_t j = 0; j < r; ++j) {
//            std::cerr << std::hex << static_cast<int>(rx[j]) << " ";
//        }
//        std::cerr << std::dec << "\n";
//    }
//}
//
//    return;
    for (int attempt = 0; attempt < kHandshakeRetries; ++attempt) {
        std::cerr << "UDP Radio handshake attempt " << (attempt + 1) << "/" << kHandshakeRetries << "...\n";

        for (int i = 0; i < 10; ++i) {
            // Send 1-byte 0xF3
            // socketlink waits for up to 10 receives with delays; emulate with timed waits
            for (int i = 0; i < kHandshakeBurstCount; ++i) {
                uint8_t nonHandshakeByte = 0xFF;
                std::cerr << m_remoteAddr.sin_addr.s_addr << ":" << ntohs(m_remoteAddr.sin_port) << "\n";
                ssize_t sent = ::sendto(
                    m_fd, &nonHandshakeByte, 1, 0,
                    reinterpret_cast<sockaddr*>(&m_remoteAddr), sizeof(m_remoteAddr)
                );
                std::cerr << "UDP Radio: Sent non-handshake byte (0xFF), sent=" << sent << "\n";

                // if (sent < 0) {
                //     std::cerr << "UDP Radio: Failed to send handshake byte. Error: " 
                //               << strerror(errno) << "\n";
                // } else {
                //     std::cerr << "UDP Radio: Handshake byte sent successfully.\n";
                // }
                // std::cerr << "UDP Radio: Sent handshake byte, sent=" << sent << "\n";
                // if (sent != 1) {
                //     // not fatal; retry
                // }


                // if (!waitReadable(m_fd, kHandshakeReplyTimeoutMs)) {
                //    continue;
                // }
        
                std::array<uint8_t, kMaxDatagram> rx{};
                sockaddr_in from{};
                socklen_t fromLen = sizeof(from);
                ssize_t r = ::recvfrom(
                    m_fd, rx.data(), rx.size(), 0,
                    reinterpret_cast<sockaddr*>(&from), &fromLen
                );
        
                if (r == 1 && rx[0] == kHandshakeByte) {
                    m_handshakeDone = true;
                    std::cerr << "Received a byte: UDP Radio handshake successful.\n";
                    //return;
                }
            }
        }
    }
    return;

    throw std::runtime_error("UdpRadio: handshake with socketlink endpoint failed (no 0xF3 reply)");
  }

bool UDPRadio::sendCrtpPacket(
    const libcrtp::CrtpLinkIdentifier * link,
    const libcrtp::CrtpPacket * packet,
    libcrtp::CrtpPacket * responsePacket)
{   
    if (!m_handshakeDone) {
        // try once more
        handshake();
      }
    
      if (!packet) return false;
    
      const uint8_t size = getPayloadSize(*packet);
      if (size > kMaxPayload) {
        std::cerr << "UdpRadio: payload too large: " << int(size) << "\n";
        return false;
      }
    
      std::array<uint8_t, kMaxDatagram> tx{};
      tx[0] = getHeader(*packet);
      if (size > 0) {
        std::memcpy(tx.data() + 1, getPayloadPtr(*packet), size);
      }
    
      const size_t txLen = static_cast<size_t>(1 + size);
    
      ssize_t sent = ::sendto(
        m_fd, tx.data(), txLen, 0,
        reinterpret_cast<sockaddr*>(&m_remoteAddr), sizeof(m_remoteAddr)
      );
    
      if (sent != static_cast<ssize_t>(txLen)) {
        return false;
      }
    
      // If caller doesn't care about response, we're done.
      if (!responsePacket) {
        return true;
      }
    
      // Wait for response (socketlink on firmware side may reply or not depending on higher layers)
      if (!waitReadable(m_fd, kRxTimeoutMs)) {
        // no response available
        return true;
      }
    
      std::array<uint8_t, kMaxDatagram> rx{};
      sockaddr_in from{};
      socklen_t fromLen = sizeof(from);
      ssize_t r = ::recvfrom(
        m_fd, rx.data(), rx.size(), 0,
        reinterpret_cast<sockaddr*>(&from), &fromLen
      );
    
      if (r <= 0) {
        return true; // treat as "no response", but send succeeded
      }
    
      setFromWire(*responsePacket, rx.data(), static_cast<size_t>(r));
      return true;

}


} // namespace libradio::udpradio