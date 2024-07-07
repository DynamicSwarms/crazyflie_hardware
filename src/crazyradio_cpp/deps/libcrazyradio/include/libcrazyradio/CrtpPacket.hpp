#pragma once

#include <stdint.h>
#include <cstring>

namespace libcrtp {
    struct CrtpPacket
    {
        uint8_t port;
        uint8_t channel;
        uint8_t data[31];
        uint8_t data_length;
    };


/*
class CrtpPacket
{
    public: 
       CrtpPacket(
            uint8_t port, 
            uint8_t channel,
            uint8_t * data, 
            uint8_t data_length)
            : m_port(port)
            , m_channel(channel)
            , m_data_length(data_length)
        {
            memcpy(m_data, data, data_length);
        }

        // Copy constructor
        CrtpPacket(CrtpPacket& other)
            : m_channel(other.getChannel())
            , m_port(other.getPort())
            , m_data_length(other.getDataLength())
        {
            memcpy(m_data, other.getData(), m_data_length);
        }

        // Move Constructor
        CrtpPacket(const CrtpPacket&& other) 
            : m_channel(other.getChannel())
            , m_port(other.getPort())
            , m_data_length(other.getDataLength())
        {
            memcpy(m_data, other.getData(), m_data_length);
        }

        virtual ~CrtpPacket();

        uint8_t getChannel() const;
        uint8_t getPort() const;
        const uint8_t * getData() const;
        uint8_t getDataLength() const;


        // Copy assignment operator
        CrtpPacket& operator=(const CrtpPacket& other) {
            if (this != &other) {
                port = other.port;
                channel = other.channel;
                if (data) delete[] data;
                length = other.length;
                if (length > 0) {
                    data = new uint8_t[length];
                    std::copy(other.data, other.data + length, data);
                } else {
                    data = nullptr;
                }
            }
            return *this;
        }

    private: 
        uint8_t m_port;
    public:
        uint8_t m_channel;
        uint8_t m_data[31];
        uint8_t m_data_length;
};
*/
}; // namespace libcrtp