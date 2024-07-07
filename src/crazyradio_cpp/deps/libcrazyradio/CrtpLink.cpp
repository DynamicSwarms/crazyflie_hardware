#include "libcrazyradio/CrtpLink.hpp"



namespace libcrtp {

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

bool CrtpLink::getPacket(CrtpPort port, CrtpPacket * packet)
{
    return m_crtpPortQueues[port].getPacket(packet);
}

bool CrtpLink::releasePacket(
    CrtpPacket * packet)
{   
    return true;
}

CrtpPort CrtpLink::getPriorityPort() const
{
    for (const auto& [port, queue] : m_crtpPortQueues) 
    {
        if (! queue.isEmtpy()) return port; 
    }
    return CrtpPort::NO_PORT;
}

uint8_t CrtpLink::getChannel() const
{
    return m_channel;
}

uint64_t CrtpLink::getAddress() const
{
    return m_address;
}

uint8_t CrtpLink::getDatarate() const
{
    return m_datarate;
}

}; // namespace libcrtp