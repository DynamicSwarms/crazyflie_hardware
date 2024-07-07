#include "libcrazyradio/CrtpLink.hpp"

#define CRTP_PORT_CONSOLE           0
#define CRTP_PORT_PARAMETERS        2
#define CRTP_PORT_COMMANDER         3
#define CRTP_PORT_MEMORY_ACCESS     4
#define CRTP_PORT_DATA_LOGGING      5
#define CRTP_PORT_LOCALIZATION      6
#define CRTP_PORT_GENERIC_SETPOINT  7
#define CRTP_PORT_PLATFORM          13
#define CRTP_PORT_CLIENT_SIDE_DEBUG 14
#define CRTP_PORT_LINK_LAYER        15


namespace libcrtp {

CrtpLink::CrtpLink(
    uint8_t channel, 
    uint64_t address,
    uint8_t datarate)
    : m_crtpPortQueues({
        {CRTP_PORT_CONSOLE,CrtpPacketQueue()},
        {CRTP_PORT_PARAMETERS,CrtpPacketQueue()},
        {CRTP_PORT_COMMANDER,CrtpPacketQueue()},
        {CRTP_PORT_MEMORY_ACCESS,CrtpPacketQueue()},
        {CRTP_PORT_DATA_LOGGING,CrtpPacketQueue()},
        {CRTP_PORT_LOCALIZATION,CrtpPacketQueue()},
        {CRTP_PORT_GENERIC_SETPOINT,CrtpPacketQueue()},
        {CRTP_PORT_PLATFORM,CrtpPacketQueue()},
        {CRTP_PORT_CLIENT_SIDE_DEBUG,CrtpPacketQueue()},
        {CRTP_PORT_LINK_LAYER,CrtpPacketQueue() }
        })
{
    m_channel = channel;
    m_address = address;
    m_datarate = datarate;
}

CrtpLink::~CrtpLink()
{
}

void CrtpLink::setRadio(
    libcrazyradio::Crazyradio * radio)
{
    radio->setChannel(m_channel);
    radio->setAddress(m_address);
    switch (m_datarate) 
    {
        case 2: 
            radio->setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_2MPS);
            break;
        case 1: 
            radio->setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_1MPS);
            break;
        default: 
            radio->setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_250KPS);
    }
}


void CrtpLink::addPacket(
    CrtpPacket * packet)
{
    m_crtpPortQueues[packet->port].addPacket(packet);
}

bool CrtpLink::getPacket(uint8_t port, CrtpPacket * packet)
{
    return m_crtpPortQueues[port].getPacket(packet);
}

bool CrtpLink::releasePacket(
    CrtpPacket * packet)
{   
    return true;
}

}; // namespace libcrtp