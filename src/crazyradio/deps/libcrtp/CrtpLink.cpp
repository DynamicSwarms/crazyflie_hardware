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
    , m_failedMessagesMaximum(100)
    , m_failedMessagesCount(0)
    , m_relaxationCountMs(0)
    , m_relaxationPeriodMs(10) // At most 100 Hz
    , m_lastSuccessfullMessageTime(0)
    , m_lastSuccessfullMessageTimeout(2000) // If 2 seconds no Communication -> Fail refardless of how many messages failed before.
{
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
    /*
        Because log messages are not obeying the ordering process and dont have a request we cannot pass them into the queue
        otherwise other requested messages would get unvalidated
        TODO: Fix this in Crazyflie Firmware, because this does not fullfill crtp specifications as defined in:
        https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/
    */
    if (packet->port == CrtpPort::DATA_LOGGING && packet->channel == 2) return false;
    
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

void CrtpLink::notifySuccessfullNullpacket()
{
    resetConnectionStats();
}

void CrtpLink::notifySuccessfullMessage(CrtpPort port)
{
    m_crtpPortQueues[port].sendPacketSuccess();
    resetConnectionStats();
}

bool CrtpLink::notifyFailedMessage()
{
    m_failedMessagesCount++;

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

void CrtpLink::tense()
{
    m_relaxationCountMs = 0;
}

void CrtpLink::relaxMs(uint8_t ms)
{
    m_relaxationCountMs += ms;
    m_lastSuccessfullMessageTime += ms;
}

bool CrtpLink::isRelaxed() const
{
    return m_relaxationCountMs >= m_relaxationPeriodMs;
}


double CrtpLink::getLinkQuality() const
{
    return 1.0 - ((double)m_failedMessagesCount / (double)m_failedMessagesMaximum);
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

void CrtpLink::resetConnectionStats()
{
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

bool CrtpLinkContainer::getRandomLink(CrtpLinkIdentifier * link) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    if (m_links.size())
    {
        auto it = m_links.begin();
        std::advance(it, rand() % m_links.size());
        linkToIdentifier(&it->second , link);
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

void CrtpLinkContainer::relaxLinks(uint8_t ms)
{

    std::unique_lock<std::mutex> mlock(m_linksMutex);
    for (auto& [key, link_] : m_links) 
    {       
        link_.relaxMs(ms);
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

bool CrtpLinkContainer::linkGetPacket(CrtpLinkIdentifier * link_id, CrtpPort port, CrtpPacket * packet)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        return link->getPacket(port, packet);
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


void CrtpLinkContainer::linkNotifySuccessfullMessage(CrtpLinkIdentifier * link_id, CrtpPort port)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        link->notifySuccessfullMessage(port);
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

double CrtpLinkContainer::linkGetLinkQuality(CrtpLinkIdentifier * link_id)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        return link->getLinkQuality();
    }
    return 0.0;
}


void CrtpLinkContainer::linkTense(CrtpLinkIdentifier * link_id)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        link->tense();
    }
}

void CrtpLinkContainer::linkRelaxMs(CrtpLinkIdentifier * link_id, uint8_t ms)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        link->relaxMs(ms);
    }
}


} // namespace libcrtp