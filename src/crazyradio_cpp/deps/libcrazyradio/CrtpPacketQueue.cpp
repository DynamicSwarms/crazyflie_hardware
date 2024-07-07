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
    CrtpPacket * packet
)
{
    m_queue.push(*packet);
}

bool CrtpPacketQueue::getPacket(
    CrtpPacket * packet)
{
    if (m_queue.empty()) return false;
    
    *packet = m_queue.front();
    m_queue.pop();
    return true;
}

}; // namespace libcrtp