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

void CrtpLink::addPacket(
    CrtpPacket * packet,
    CrtpResponseCallback  callback)
{
    m_crtpPortQueues[packet->port].addPacket(packet, callback);
}

bool CrtpLink::getPacket(
    CrtpPort port, 
    CrtpPacket * packet)
{
    return m_crtpPortQueues[port].getPacket(packet);
}

bool CrtpLink::releasePacket(
    CrtpPacket * packet,
    CrtpResponseCallback &  callback)
{   
    return m_crtpPortQueues[packet->port].releasePacket(packet, callback);
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



CrtpLinkContainer::CrtpLinkContainer() 
    : m_links()
{
}

CrtpLinkContainer::~CrtpLinkContainer()
{
    /* Maybe have to close links properly */
}

void CrtpLinkContainer::addLink(uint8_t channel, uint64_t address, uint8_t datarate)
{
    CrtpLink link(channel, address, datarate);
    std::pair<uint8_t, uint64_t> linkKey = {channel,address};
    m_links.insert({linkKey, link}); // If already in m_links this wont duplicate
}

bool CrtpLinkContainer::getLink(CrtpLink ** link, uint8_t channel, uint64_t address)
{
    std::pair<uint8_t, uint64_t> linkKey = {channel, address};
    auto link_ = m_links.find(linkKey); // A sad cpp construct
    if (link_ != m_links.end()) {
        *link = &link_->second;
        return true;
    }
    return false;
}

bool CrtpLinkContainer::getHighestPriorityLink(CrtpLink ** link, CrtpPort * port)
{
    libcrtp::CrtpPort highestPriorityPort = libcrtp::CrtpPort::NO_PORT;
    std::pair<uint8_t, uint64_t> bestKey = {0,0};
    for (const auto& [key, link_] : m_links) 
    {       
        libcrtp::CrtpPort port = link_.getPriorityPort();
        if (port < highestPriorityPort) {
            highestPriorityPort = port;
            bestKey = key;
        } 
    }
    auto link_ = m_links.find(bestKey);
    if (highestPriorityPort != libcrtp::CrtpPort::NO_PORT && link_ != m_links.end()) 
    {   
        *link = &link_->second;
        *port = highestPriorityPort;
        return true;
    }
    return false;
}

bool CrtpLinkContainer::getRandomLink(CrtpLink ** link)
{
    if (m_links.size())
    {
        auto it = m_links.begin();
        std::advance(it, rand() % m_links.size());
        *link = &it->second;
        return true;
    }
    return false;
}




}; // namespace libcrtp