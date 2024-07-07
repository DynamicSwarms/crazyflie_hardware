#include "libcrazyradio/CrtpLink.hpp"



namespace libcrtp {

enum CrtpPort {
    CONSOLE             = 0,
    PARAMETERS          = 2,
    COMMANDER           = 3,
    MEMORY_ACCESS       = 4,
    DATA_LOGGING        = 5,
    LOCALIZATION        = 6,
    GENERIC_SETPOINT    = 7,
    PLATFORM            = 13,
    CLIENT_SIDE_DEBUG   = 14,
    LINK_LAYER          = 15,
    NO_PORT             = 0xFF
};

CrtpLink::CrtpLink(
    uint8_t channel, 
    uint64_t address,
    uint8_t datarate)
    : m_crtpPortQueues({
        {CONSOLE,           CrtpPacketQueue()},
        {PARAMETERS,        CrtpPacketQueue()},
        {COMMANDER,         CrtpPacketQueue()},
        {MEMORY_ACCESS,     CrtpPacketQueue()},
        {DATA_LOGGING,      CrtpPacketQueue()},
        {LOCALIZATION,      CrtpPacketQueue()},
        {GENERIC_SETPOINT,  CrtpPacketQueue()},
        {PLATFORM,          CrtpPacketQueue()},
        {CLIENT_SIDE_DEBUG, CrtpPacketQueue()},
        {LINK_LAYER,        CrtpPacketQueue() }
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

uint8_t CrtpLink::getPriorityPort() 
{
    for (auto const& portQueue : m_crtpPortQueues) 
    {
        // portQueue is tuple of [key: port, val: crtp_queue]
        if (! portQueue.second.isEmtpy()) return portQueue.first; 
    }
    return CrtpPort::NO_PORT;
}

}; // namespace libcrtp