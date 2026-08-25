#include "libcrtp/CrtpLinkContainer.hpp"
#include <cstdlib>
namespace libcrtp {

CrtpLinkContainer::CrtpLinkContainer() 
    : m_links()
    , m_randomGenerator(std::random_device{}())
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
    libcrtp::CrtpPort highestBroadcastPriority = libcrtp::CrtpPort::NO_PORT;
    libcrtp::CrtpPort highestPriority = libcrtp::CrtpPort::NO_PORT;
    std::vector<const CrtpLink*> broadcastCandidates;
    std::vector<const CrtpLink*> candidates;

    for (const auto& [key, link_] : m_links) 
    {       
        const libcrtp::CrtpPort candidatePort = link_.getPriorityPort();
        if (candidatePort == libcrtp::CrtpPort::NO_PORT) continue;

        auto& candidatePriority = link_.isBroadcast() ? highestBroadcastPriority : highestPriority;
        auto& candidateLinks = link_.isBroadcast() ? broadcastCandidates : candidates;
        
        if (candidatePort < candidatePriority) {
            candidatePriority = candidatePort;
            candidateLinks.clear();
        }
        if (candidatePort == candidatePriority) candidateLinks.push_back(&link_);
    }

    const auto& selectedCandidates = broadcastCandidates.empty() ? candidates : broadcastCandidates;
    if (selectedCandidates.empty()) return false;

    const CrtpLink* selected = selectedCandidates[randomIndex(selectedCandidates.size())];
    linkToIdentifier(selected, link);
    *port = broadcastCandidates.empty() ? highestPriority : highestBroadcastPriority;
    return true;
}

bool CrtpLinkContainer::getRandomRelaxedNonBroadcastLink(CrtpLinkIdentifier * link) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    // Filter for relaxed links
    std::vector<libcrtp::CrtpLinkIdentifier> relaxed_links;

    libcrtp::CrtpLinkIdentifier link_id;
    // Iterate over the original map
    for (const auto& entry : m_links) {
        if (entry.second.isRelaxed() &&
            !entry.second.isBroadcast()) {
            linkToIdentifier(&entry.second, &link_id);
            relaxed_links.push_back(link_id);
        }
    }

    if (relaxed_links.size())
    {
        copyLinkIdentifier(&relaxed_links[randomIndex(relaxed_links.size())], link);
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

bool CrtpLinkContainer::linkGetOutboundPacket(
    CrtpLinkIdentifier * link_id,
    CrtpPacket * packet,
    bool * isPortPacket)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        link->getOutboundPacket(packet, isPortPacket);
        return true;
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

void CrtpLinkContainer::linkNotifySuccessfullNullpacket(CrtpLinkIdentifier * link_id, bool responseIsNullpacket)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        link->notifySuccessfullNullpacket(responseIsNullpacket);
    }
}


void CrtpLinkContainer::linkNotifySuccessfullPortMessage(CrtpLinkIdentifier * link_id, CrtpPort port, bool responseIsNullpacket)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        link->notifySuccessfullPortMessage(port, responseIsNullpacket);
    }
}



bool CrtpLinkContainer::linkNotifyFailedNullpacket(CrtpLinkIdentifier * link_id)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        return link->notifyFailedNullpacket();
    }
    return false;
}

bool CrtpLinkContainer::linkNotifyFailedPortMessage(CrtpLinkIdentifier * link_id)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        return link->notifyFailedPortMessage();
    }
    return false;
}

size_t CrtpLinkContainer::randomIndex(size_t size) const
{
    std::uniform_int_distribution<size_t> distribution(0, size - 1);
    return distribution(m_randomGenerator);
}

} // namespace libcrtp
