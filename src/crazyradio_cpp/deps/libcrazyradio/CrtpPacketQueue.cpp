#include "libcrazyradio/CrtpPacketQueue.hpp"

#include <iostream>

namespace libcrtp {

CrtpPacketQueue::CrtpPacketQueue()
    : m_queue()
{   
    
}

CrtpPacketQueue::~CrtpPacketQueue()
{

}

void CrtpPacketQueue::addPacket(
    CrtpPacket * packet,
    CrtpResponseCallback  callback
)
{
    m_queue.push(std::make_pair(*packet, callback));
}

bool CrtpPacketQueue::getPacket(
    CrtpPacket * packet,
    CrtpResponseCallback  & callback)
{
    if (m_queue.empty()) return false;
    auto pair = m_queue.front();
    *packet = pair.first;
    callback = pair.second; 
    m_queue.pop();
    return true;
}

bool CrtpPacketQueue::isEmtpy() const
{
    return m_queue.empty();
}

}; // namespace libcrtp