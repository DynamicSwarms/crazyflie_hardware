#include "udpradio/Firmwarelink.hpp"

#include <fcntl.h>
#include <iostream>
#include <ostream>

namespace libradio::firmwarelink
{

constexpr int kMaxPayload = 31;                 // CRTP max data size (excluding header)
constexpr int kMaxDatagram = 1 + kMaxPayload;   // header + payload

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

Firmwarelink::Firmwarelink(uint16_t port)
{
    m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (m_fd < 0) {
        throw std::runtime_error("UdpRadio: socket() failed");
    }

    m_in_address.sin_family = AF_INET;
    m_in_address.sin_addr.s_addr = INADDR_ANY;
    m_in_address.sin_port = htons(port);

    if (bind(m_fd, reinterpret_cast<sockaddr*>(&m_in_address), sizeof(m_in_address)) < 0) {
        close(m_fd);
        m_fd = -1;
        throw std::runtime_error("UdpRadio: bind() failed");
    }

    // Set socket to non-blocking mode
    int flags = fcntl(m_fd, F_GETFL, 0);
    if (flags < 0) {
        throw std::runtime_error("fcntl(F_GETFL) failed");
    }

    if (fcntl(m_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        throw std::runtime_error("fcntl(F_SETFL) failed");
    }
}

Firmwarelink::~Firmwarelink()
{
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
    std::cerr << "Firmwarelink stopped." << std::endl;
}

int
Firmwarelink::send(
    const uint8_t* data, size_t size,
    uint8_t* receive_data)
{
    if (m_is_connected) {
        //std::cerr << "UDP Radio: Sending " << size << std::endl;   
        ssize_t sent = sendto(
            m_fd, data, size, 0,
            reinterpret_cast<const sockaddr*>(&m_out_address), sizeof(m_out_address)
        );
        //std::cerr << "UDP Radio: sendto() returned " << sent << std::endl;
    
        if (sent < 0) {
            std::cerr << "UDP Radio: sendto() failed\n";
        } else {
            //std::cerr << "UDP Radio: Sent " << sent << " bytes\n";
        }
    }

    std::array<uint8_t, kMaxDatagram> rx{};
    
    ssize_t r = recvfrom(
        m_fd, receive_data, kMaxDatagram, 0,
        reinterpret_cast<sockaddr*>(&m_out_address), &m_address_len
    );
    //std::cerr << "UDP Radio: recvfrom() returned " << r << std::endl;

    if (r > 0) m_is_connected = true;
    return r;
}

}