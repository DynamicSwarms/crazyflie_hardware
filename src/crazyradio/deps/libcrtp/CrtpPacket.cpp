#include "libcrazyradio/CrtpPacket.hpp"


namespace libcrtp {



CrtpPacket::~CrtpPacket() {}


uint8_t CrtpPacket::getChannel() const
{
    return m_channel;
}

uint8_t CrtpPacket::getPort() const
{
    return m_port;
}

const uint8_t * CrtpPacket::getData() const
{
    return m_data;
}

uint8_t CrtpPacket::getDataLength() const
{
    return m_data_length;
}

} // namespace libcrtp