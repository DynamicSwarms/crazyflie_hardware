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
    , m_isBroadcast(((address >> 4 * 8) & 0xFF) == 0xFF) // Broadcasting Links have 0xFF as the first byte of the address (cfs have 0xE7)   
    , m_failedMessagesMaximum(100) // After this many failed messages we consider the link dead, we also wait m_failedMessageRetryTimeout before retrying
    , m_nullpacketRelaxationMs(10) // Wait at least 10 ms if a nullpacket was received (probably CF doesnt want to talk right now)
    , m_lastSuccessfullMessageTimeoutMs(2000) // If 2 seconds no Communication -> Fail refardless of how many messages failed before.
    , m_minimumBackoffMs(10)
    , m_maximumBackoffMs(100)
    , m_failedMessagesCount(0)
    , m_backoffTimer(m_minimumBackoffMs)
    , m_relaxationTimer(m_nullpacketRelaxationMs)
    , m_livenessTimer(m_lastSuccessfullMessageTimeoutMs)
    , m_randomGenerator(std::random_device{}())
    , m_linkQuality(~0) // 64 bits of failed and successful messages (bits)
    , m_currentOutbound(std::nullopt)
{
    m_livenessTimer.reset();
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

void CrtpLink::getOutboundPacket(CrtpPacket * packet, bool * isPortPacket)
{
    if (m_currentOutbound.has_value()) {
        *packet = m_currentOutbound->packet;
        *isPortPacket = m_currentOutbound->isPortPacket;
        return;
    }

    CrtpPort port = getPriorityPort();
    if (port != CrtpPort::NO_PORT && m_crtpPortQueues[port].getPacket(packet)) {
        m_currentOutbound = CurrentOutbound{*packet, true};
        *isPortPacket = true;
        return;
    }

    *packet = nullPacket;
    m_currentOutbound = CurrentOutbound{*packet, false};
    *isPortPacket = false;
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
    if (!m_backoffTimer.isReady())
        return CrtpPort::NO_PORT; // Wait before sending a failed packet again.

    for (const auto& [port, queue] : m_crtpPortQueues) 
    {
        if (! queue.isEmtpy()) return port; 
    }
    return CrtpPort::NO_PORT;
}

void CrtpLink::notifySuccessfullNullpacket(bool responseIsNullpacket)
{
    m_currentOutbound.reset();
    if (responseIsNullpacket) m_relaxationTimer.reset();
    onSuccessfullMessage();
}

void CrtpLink::notifySuccessfullPortMessage(CrtpPort port, bool responseIsNullpacket)
{
    m_currentOutbound.reset();
    if (responseIsNullpacket) m_relaxationTimer.reset();
    m_crtpPortQueues[port].sendPacketSuccess();
    onSuccessfullMessage();
}

bool CrtpLink::notifyFailedNullpacket()
{
    resetBackoffTimer();
    return onFailedMessage();
}

bool CrtpLink::notifyFailedPortMessage()
{   
    resetBackoffTimer();
    return onFailedMessage();
}

/**
 *     failures   exponent   backoff range
 *        0          0       [min,      min]
 *        1          1       [min,  2 * min]
 *        2          1       [min,  2 * min]
 *        3          2       [min,  4 * min]
 *        4          2       [min,  4 * min]
 *        5          3       [min,  8 * min]
 *        6          3       [min,  8 * min]
 *        7+         4       [min, 16 * min]
 *     
 *  Always bounded by m_maximumBackoffMs !
 */
void CrtpLink::resetBackoffTimer()
{
    const uint32_t exponent = std::min<uint32_t>((m_failedMessagesCount + 1)/2, 4);
    const uint32_t maximumMs = std::min(m_maximumBackoffMs, m_minimumBackoffMs << exponent);
    std::uniform_int_distribution<uint32_t> distribution(m_minimumBackoffMs, maximumMs);
    m_backoffTimer.reset(distribution(m_randomGenerator));
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
    m_backoffTimer.tick(ms);
    m_relaxationTimer.tick(ms);
    m_livenessTimer.tick(ms);
}

bool CrtpLink::isRelaxed() const
{
    return m_backoffTimer.isReady() && m_relaxationTimer.isReady();
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
    m_livenessTimer.reset();
}

bool CrtpLink::onFailedMessage()
{
    m_linkQuality <<= 1; // shift left, add a 0 to the end of the bitfield

    m_failedMessagesCount++;

    /**
     * Fail after m_failedMessagesMaximum or if lastSuccessfullMessage > m_lastSuccessfullMessageTimeout
    */
    if (m_failedMessagesCount > m_failedMessagesMaximum
        || m_livenessTimer.isReady())
    {
        return true;
    }
    return false;
}

} // namespace libcrtp
