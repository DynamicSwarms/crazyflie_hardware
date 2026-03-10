#pragma once 
#include <stdint.h>
#include <array>
#include <cstddef>

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
#include <queue>
#include <memory>

#include "libcrtp/CrtpPacket.hpp"


namespace libradio::sitllink {

class SITLlink {
public:
    SITLlink(uint16_t port);
    ~SITLlink();

    void send(
        const libcrtp::CrtpPacket * packet, 
        libcrtp::CrtpPacket * responsePacket,
        bool is_broadcast);
    
    bool is_connected() const { return m_is_connected; }
private: 
    ssize_t socket_transfer(
        const uint8_t* data, const size_t size,
        uint8_t* receive_data);

    void data_to_packet(
        const uint8_t* data, const size_t size,
        libcrtp::CrtpPacket * packet);
    
    void packet_to_data(
        const libcrtp::CrtpPacket * packet,
        uint8_t* data, size_t* size);

    bool is_null_packet(const libcrtp::CrtpPacket * packet) const {
        return (packet->port == libcrtp::CrtpPort::LINK_LAYER && packet->channel == 3);
    }

private:
    int m_fd{-1};
    
    struct sockaddr_in m_in_address;
    struct sockaddr_in m_out_address;
    socklen_t m_address_len{sizeof(m_out_address)};

    bool m_is_connected{false};   
    
    std::shared_ptr<std::queue<libcrtp::CrtpPacket>> m_in_packets;
};

} // namespace libradio::sitllink