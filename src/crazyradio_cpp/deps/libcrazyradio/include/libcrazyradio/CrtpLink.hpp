#pragma once

#include <stdint.h>

#include <map>

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
        *  Adds a to be sent out Packet to the link. 
        */
        void addPacket(CrtpPacket * packet,  CrtpResponseCallback  callback);

        bool getPacket(CrtpPort port, CrtpPacket * packet);

        /**
        *  Gets a crazyflies response, which should be crossreferenced to an yet unacknowledged request which excpected a response. 
        *  Returns False if nobody listened for a response.
        */
        bool releasePacket(CrtpPacket * packet, CrtpResponseCallback & callback);

        /**
         * Returns the port with highest priority with a packet to send inside.
         * Returns CrtpPort::NO_PORT if completely empty
         */
        CrtpPort getPriorityPort() const;


        uint8_t getChannel() const;
        uint64_t getAddress() const;
        uint8_t getDatarate() const;
        bool isBroadcast() const;

    private: 
        uint8_t m_channel;
        uint64_t m_address;
        uint8_t m_datarate; 
        bool m_isBroadcast;

        std::map<CrtpPort, CrtpPacketQueue> m_crtpPortQueues;
};

class CrtpLinkContainer
{
    public:
        CrtpLinkContainer();
        virtual ~CrtpLinkContainer();

        /**
         * Adds Link to Container, if already present will not overwrite
         */
        void addLink(uint8_t channel, uint64_t address, uint8_t datarate);

        bool getLink(CrtpLink ** link, uint8_t channel, uint64_t address);

        bool getHighestPriorityLink(CrtpLink ** link, CrtpPort * port);

        bool getRandomLink(CrtpLink ** link);

    private: 
        std::map<std::pair<uint8_t, uint64_t>, libcrtp::CrtpLink> m_links;
};

}; // namespace libcrtp