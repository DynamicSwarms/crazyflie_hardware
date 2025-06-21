#include "libcrtp/CrtpLink.hpp"
#include <iostream>
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
    , m_channel(channel)
    , m_address(address)
    , m_datarate(datarate)
    , m_isBroadcast(((address >> 4 * 8) & 0xFF) == 0xFF) // Broadcasting Packet if 0xFF   
    , m_failedMessagesMaximum(30) // After this many failed messages we consider the link dead, we also wait 10 ms for a retry
    , m_failedMessagesCount(0)
    , m_relaxationCountMs(0)
    , m_relaxationPeriodMs(10) // At most 100 Hz for ping messages
    , m_lastSuccessfullMessageTime(0)
    , m_lastSuccessfullMessageTimeout(2000) // If 2 seconds no Communication -> Fail refardless of how many messages failed before.
    , m_failedMessageRetryTimeout(30) // If a message fails, we wait 30 ms before retrying
    , m_linkQuality(~0) // 64 bits of failed and successful messages (bits)
{
}

CrtpLink::~CrtpLink()
{
}

void CrtpLink::addPacket(
    CrtpPacket * packet,
    CrtpResponseCallback  callback)
{
    //if (packet->expectsResponse) 
    //    std::cerr << "Add: " << (int)packet->port << std::endl;
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
    /*
        Because log messages are not obeying the ordering process and dont have a request we cannot pass them into the queue
        otherwise other requested messages would get unvalidated
        TODO: Fix this in Crazyflie Firmware, because this does not fullfill crtp specifications as defined in:
        https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/
    */
    if (packet->port == CrtpPort::DATA_LOGGING && packet->channel == 2) return false;
    
    bool released = m_crtpPortQueues[packet->port].releasePacket(packet, callback);
    //if (released)
    //    std::cerr << "Released?: " << (int)packet->port << (released ? "yes" : "no") << std::endl;
    return released;
}

CrtpPort CrtpLink::getPriorityPort() const
{
    if (m_lastSuccessfullMessageTime < m_failedMessageRetryTimeout * m_failedMessagesCount)
        return CrtpPort::NO_PORT; // Wait before sending a failed packet again.

    for (const auto& [port, queue] : m_crtpPortQueues) 
    {
        if (! queue.isEmtpy()) return port; 
    }
    return CrtpPort::NO_PORT;
}

void CrtpLink::notifySuccessfullNullpacket()
{
    m_relaxationCountMs = 0;
    onSuccessfullMessage();
}

void CrtpLink::notifySuccessfullPortMessage(CrtpPort port)
{
    m_crtpPortQueues[port].sendPacketSuccess();
    onSuccessfullMessage();
}

bool CrtpLink::notifyFailedMessage()
{
    m_failedMessagesCount++;
    m_linkQuality <<= 1; // shift left, add a 0 to the end of the bitfield

    /**
     * Fail after m_failedMessagesMaximum or if lastSuccessfullMessage > m_lastSuccessfullMessageTimeout
    */
    if (m_failedMessagesCount > m_failedMessagesMaximum || m_lastSuccessfullMessageTime > m_lastSuccessfullMessageTimeout) 
    {
        return true;
    }
    return false;
}

void CrtpLink::retrieveAllCallbacks(std::vector<CrtpResponseCallback>& callbacks)
{
    for (auto& [port, queue] : m_crtpPortQueues) 
    {
        queue.retrieveAllCallbacks(callbacks); 
    }
}

void CrtpLink::tickMs(uint8_t ms)
{
    m_relaxationCountMs += ms;
    m_lastSuccessfullMessageTime += ms;
}

bool CrtpLink::isRelaxed() const
{
    if (m_failedMessagesCount) // If we have failed messages, we should not send nullpackets
        for (const auto& [port, queue] : m_crtpPortQueues) if (! queue.isEmtpy()) return false; 
    
    return m_relaxationCountMs >= m_relaxationPeriodMs;
}


double CrtpLink::getLinkQuality() const
{
    uint64_t x = m_linkQuality;
    int count = 0;
    while (x) {
        x &= (x - 1);
        count++;
    }
    return count / 64.0;
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

bool CrtpLink::isBroadcast() const
{
    return m_isBroadcast;
}

void CrtpLink::onSuccessfullMessage()
{
    m_linkQuality = (m_linkQuality << 1) | 1; // shift left, add a 1 to the end of the bitfield
    m_failedMessagesCount = 0;
    m_lastSuccessfullMessageTime = 0;
}





CrtpLinkContainer::CrtpLinkContainer() 
    : m_links()
{
}

CrtpLinkContainer::~CrtpLinkContainer()
{
    /* Maybe have to close links properly */
}

void CrtpLinkContainer::copyLinkIdentifier(CrtpLinkIdentifier * from_link, CrtpLinkIdentifier * to_link) const
{
    to_link->channel = from_link->channel;
    to_link->address = from_link->address;
    to_link->datarate = from_link->datarate;
    to_link->isBroadcast = from_link->isBroadcast;

}

void CrtpLinkContainer::linkToIdentifier(const CrtpLink * link, CrtpLinkIdentifier *  link_id) const
{
    link_id->channel = link->getChannel();
    link_id->address = link->getAddress();
    link_id->datarate = link->getDatarate();
    link_id->isBroadcast = link->isBroadcast();
}

bool CrtpLinkContainer::linkFromIdentifier(CrtpLink ** link, CrtpLinkIdentifier * link_id)
{
    std::pair<uint8_t, uint64_t> key = {link_id->channel,link_id->address};
    auto link_ = m_links.find(key);
    if (link_ != m_links.end()) 
    {   
        *link = &link_->second;
        return true;
    }
    return false; 
}

void CrtpLinkContainer::addLink(uint8_t channel, uint64_t address, uint8_t datarate)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink link(channel, address, datarate);
    std::pair<uint8_t, uint64_t> linkKey = {channel,address};
    m_links.insert({linkKey, link}); // If already in m_links this wont duplicate
}

bool CrtpLinkContainer::removeLink(CrtpLinkIdentifier * link_id, std::vector<CrtpResponseCallback>& callbacks)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    std::pair<uint8_t, uint64_t> linkKey = {link_id->channel,link_id->address};
    auto link = m_links.find(linkKey);
    if (link != m_links.end()) 
    {
        link->second.retrieveAllCallbacks(callbacks);
        m_links.erase(link);
        return true;
    }
    return false;
}

bool CrtpLinkContainer::getLinkIdentifier(CrtpLinkIdentifier * link, uint8_t channel, uint64_t address) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    std::pair<uint8_t, uint64_t> linkKey = {channel, address};
    auto link_ = m_links.find(linkKey); // A sad cpp construct
    if (link_ != m_links.end()) {
        linkToIdentifier(&link_->second , link);
        return true;
    }
    return false;
}

bool CrtpLinkContainer::getHighestPriorityLink(CrtpLinkIdentifier * link, CrtpPort * port) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    libcrtp::CrtpPort highestPriorityPort = libcrtp::CrtpPort::NO_PORT;
    std::pair<uint8_t, uint64_t> bestKey = {0,0};
    for (const auto& [key, link_] : m_links) 
    {       
        libcrtp::CrtpPort port = link_.getPriorityPort();
        if (link_.isBroadcast() && port != libcrtp::CrtpPort::NO_PORT) {
            // If there is a broadcast Packet. Send immediately.
            highestPriorityPort = port;
            bestKey = key;
            break;
        }
        if (port < highestPriorityPort) {
            highestPriorityPort = port;
            bestKey = key;
        } 
    }
    auto link_ = m_links.find(bestKey);
    if (highestPriorityPort != libcrtp::CrtpPort::NO_PORT && link_ != m_links.end()) 
    {   
        linkToIdentifier(&link_->second , link);
        *port = highestPriorityPort;
        return true;
    }
    return false;
}

bool CrtpLinkContainer::getRandomRelaxedNonBroadcastLink(CrtpLinkIdentifier * link) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    // Filter for relaxed links
    std::vector<libcrtp::CrtpLinkIdentifier> relaxed_links;

    libcrtp::CrtpLinkIdentifier link_id;
    // Iterate over the original map
    for (const auto& entry : m_links) {
        if (entry.second.isRelaxed() && !entry.second.isBroadcast()) {
            linkToIdentifier(&entry.second, &link_id);
            relaxed_links.push_back(link_id);
        }
    }

    if (relaxed_links.size())
    {
        auto it = relaxed_links.begin();
        std::advance(it, rand() % relaxed_links.size());
        copyLinkIdentifier(&(*it), link);
        return true;
    }
    return false;

}

void CrtpLinkContainer::tickLinksMs(uint8_t ms)
{

    std::unique_lock<std::mutex> mlock(m_linksMutex);
    for (auto& [key, link_] : m_links) 
    {       
        link_.tickMs(ms);
    }
}

void CrtpLinkContainer::getConnectionStats(std::vector<CrtpLinkIdentifier>& links, std::vector<double>& quality) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    links.clear();
    quality.clear();
    for (const auto& [key, link_] : m_links) 
    {       
        CrtpLinkIdentifier link_id;
        linkToIdentifier(&link_, &link_id);
        links.push_back(link_id);
        quality.push_back(link_.getLinkQuality());
    }
} 



void CrtpLinkContainer::linkAddPacket(CrtpLinkIdentifier * link_id, CrtpPacket * packet, CrtpResponseCallback callback)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        link->addPacket(packet, callback);
    }
}

bool CrtpLinkContainer::linkGetHighestPriorityPacket(CrtpLinkIdentifier * link_id, CrtpPacket * packet)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        CrtpPort port = link->getPriorityPort();
        if (port == CrtpPort::NO_PORT) return false; // No packets available
        return link->getPacket(port, packet);
    }
    return false;
}

bool CrtpLinkContainer::linkReleasePacket(CrtpLinkIdentifier * link_id, 
                                          CrtpPacket * responsePacket, 
                                          CrtpResponseCallback & callback)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        return link->releasePacket(responsePacket, callback);
    }
    return false;
}

void CrtpLinkContainer::linkNotifySuccessfullNullpacket(CrtpLinkIdentifier * link_id)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        link->notifySuccessfullNullpacket();
    }
}


void CrtpLinkContainer::linkNotifySuccessfullPortMessage(CrtpLinkIdentifier * link_id, CrtpPort port)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        link->notifySuccessfullPortMessage(port);
    }
}



bool CrtpLinkContainer::linkNotifyFailedMessage(CrtpLinkIdentifier * link_id)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        return link->notifyFailedMessage();
    }
    return false;
}

} // namespace libcrtp