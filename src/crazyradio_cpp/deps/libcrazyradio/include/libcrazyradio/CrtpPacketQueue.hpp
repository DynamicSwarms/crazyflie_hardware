#pragma once

#include <queue>

#include "CrtpPacket.hpp"

class CrtpPacketQueue
{

    public: 
        PacketQueue(
            uint8_t crtp_channel
        );

        virtual ~PacketQueue();

        void addPacket(CrtpPacket packet);

        bool getPacket();


    private: 
        uint8_t m_crtp_channel;
        std::queue<CrtpPacket> queue;
        
};