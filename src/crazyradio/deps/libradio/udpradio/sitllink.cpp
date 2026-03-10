#include "udpradio/sitllink.hpp"

#include <fcntl.h>
#include <iostream>
#include <ostream>

namespace libradio::sitllink
{

constexpr int kMaxPayload = 31;                 // CRTP max data size (excluding header)
constexpr int kMaxDatagram = 1 + kMaxPayload;   // header + payload

SITLlink::SITLlink(uint16_t port)
: m_in_packets(std::make_shared<std::queue<libcrtp::CrtpPacket>>())
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

SITLlink::~SITLlink()
{
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
    std::cerr << "Firmwarelink stopped." << std::endl;
}

void 
SITLlink::send(
    const libcrtp::CrtpPacket * packet, 
    libcrtp::CrtpPacket * responsePacket,
    bool is_broadcast)
{
    uint8_t buf[kMaxDatagram];
    size_t size;
    packet_to_data(packet, buf, &size);

    uint8_t receive_buf[kMaxDatagram];
    ssize_t recv_size = socket_transfer(buf, size, receive_buf);

    // On broadcast we receive packets, but cannot return them
    // therefore queue all received packets and return if possible
    // If no proper packets are available, return a null packet
    if (recv_size > 0)
    {
        data_to_packet(receive_buf, recv_size, responsePacket);
        if (!is_null_packet(responsePacket)) m_in_packets->push(*responsePacket);

        if (is_broadcast) memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
        else {
            if (!m_in_packets->empty()) {
                *responsePacket = m_in_packets->front();
                m_in_packets->pop();
            } else {
                memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
            }  
        }
    } else { // No packet received, return null packet
        memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
    }
}

ssize_t
SITLlink::socket_transfer(
    const uint8_t* data, size_t size,
    uint8_t* receive_data)
{
    if (m_is_connected) {
        ssize_t sent = sendto(
            m_fd, data, size, 0,
            reinterpret_cast<const sockaddr*>(&m_out_address), sizeof(m_out_address)
        );
        
        if (sent < 0) {
            std::cerr << "UDP Radio: sendto() failed\n";
        }
    }

    ssize_t r = recvfrom(
        m_fd, receive_data, kMaxDatagram, 0,
        reinterpret_cast<sockaddr*>(&m_out_address), &m_address_len
    );

    if (r > 0) m_is_connected = true;
    return r;
}

void
SITLlink::data_to_packet(const uint8_t* data, const size_t size,
               libcrtp::CrtpPacket * packet)
{
    packet->channel = data[0] & 0b11;
    packet->port = (libcrtp::CrtpPort)(data[0] >> 4 & 0xF);
    memcpy(packet->data, &data[1], size - 1);
    packet->dataLength = size - 1;
}

void 
SITLlink::packet_to_data(const libcrtp::CrtpPacket * packet,
               uint8_t* data, size_t* size)
{
    data[0] = packet->port << 4 | packet->channel;
    memcpy(&data[1], packet->data, packet->dataLength);
    *size = 1 + packet->dataLength;
}

}