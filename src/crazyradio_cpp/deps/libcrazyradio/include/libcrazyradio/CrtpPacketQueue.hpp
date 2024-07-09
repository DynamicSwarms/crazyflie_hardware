#pragma once

#include <stdint.h>
#include <queue>
#include <functional>

#include "CrtpPacket.hpp"

namespace libcrtp {


class CrtpPacketQueue
{

    public: 
        CrtpPacketQueue();

        virtual ~CrtpPacketQueue();

        void addPacket(CrtpPacket * packet, CrtpResponseCallback  callback);

        bool getPacket(CrtpPacket * packet, CrtpResponseCallback  & callback);

        bool isEmtpy() const;

    private: 
        std::queue<std::pair<CrtpPacket, CrtpResponseCallback>> m_queue;
};

}; // namespace libcrtp