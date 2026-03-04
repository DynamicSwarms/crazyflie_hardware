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


namespace libradio::firmwarelink {

    class Firmwarelink {
    public:
        Firmwarelink(uint16_t port);
        ~Firmwarelink();

        int send(const uint8_t* data, size_t size, uint8_t* receive_data);
        size_t receive(uint8_t* buffer, size_t bufferSize);

    private:
        int m_fd{-1};
        
        struct sockaddr_in m_in_address;
        struct sockaddr_in m_out_address;
        socklen_t m_address_len{sizeof(m_out_address)};

        bool m_is_connected{false};    
    };

} // namespace libradio::firmwarelink