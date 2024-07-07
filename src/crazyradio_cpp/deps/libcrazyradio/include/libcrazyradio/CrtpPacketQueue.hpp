#pragma once

#include <stdint.h>
#include <queue>

#include "CrtpPacket.hpp"

namespace libcrtp {

class CrtpPacketQueue
{

    public: 
        CrtpPacketQueue();

        virtual ~CrtpPacketQueue();

        void addPacket(CrtpPacket * packet);

        bool getPacket(CrtpPacket * packet);

        bool isEmtpy() const;

    private: 
        std::queue<CrtpPacket> m_queue;
};

}; // namespace libcrtp