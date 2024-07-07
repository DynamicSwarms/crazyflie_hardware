#pragma once

#include <stdint.h>

#include <map>

#include "libcrazyradio/Crazyradio.hpp"
#include "libcrazyradio/CrtpPacketQueue.hpp"
#include "libcrazyradio/CrtpPacket.hpp"

namespace libcrtp {

class CrtpLink
{
    public: 
       


        CrtpLink(
            uint8_t channel,
            uint64_t address,
            uint8_t datarate
        );

        virtual ~CrtpLink(); 

        /** 
        *   Sets radio to the link's specific channel, address and datarate 
        */
        void setRadio(libcrazyradio::Crazyradio * radio);
    
        /**
        *  Adds a to be sent out Packet to the link. 
        */
        void addPacket(CrtpPacket * packet);

        bool getPacket(uint8_t port, CrtpPacket * packet);

        /**
        *  Gets a crazyflies response, which should be crossreferenced to an yet unacknowledged request which excpected a response. 
        *  Returns False if nobody listened for a response.
        */
        bool releasePacket(CrtpPacket * packet);

        /**
         * Returns the port with highest priority with a packet to send inside.
         * Returns CrtpPort::NO_PORT if completely empty
         */
        uint8_t getPriorityPort();

    private: 
        uint8_t m_channel;
        uint64_t m_address;
        uint8_t m_datarate; 

        std::map<uint8_t, CrtpPacketQueue> m_crtpPortQueues;
};

}; // namespace libcrtp