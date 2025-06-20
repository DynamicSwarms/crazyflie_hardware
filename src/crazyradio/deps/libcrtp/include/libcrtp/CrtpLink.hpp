#pragma once

#include <stdint.h>

#include <map>
#include <mutex>

#include "libcrtp/CrtpPacketQueue.hpp"
#include "libcrtp/CrtpPacket.hpp"

namespace libcrtp {
struct CrtpLinkIdentifier
{
    uint8_t channel;
    uint64_t address;
    uint8_t datarate;
    bool isBroadcast;
};

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
        
        /**
         * A nullpacket from polling was successfully sent. 
         * Reset connection stats.
        */
        void notifySuccessfullNullpacket();

        /**
         * Will remove the message from the Port because it was successfully sent out. 
        */
        void notifySuccessfullMessage(CrtpPort port);
        /*Notifies about a failed send attempt, returns true if link shall die*/
        bool notifyFailedMessage();

        /**
         * This is called before decontrstuction.
         * This way the callbacks can be returned with false
        */
        void retrieveAllCallbacks(std::vector<CrtpResponseCallback>& callbacks);



        /**
         * Relax and tense the link.
         * This ensures that the rate of nullpackets is limited. 
        */
        void tense();
        void relaxMs(uint8_t ms);

        // Check if the link is relaxed and nullpacket can be sent
        bool isRelaxed() const;

        double getLinkQuality() const;

        uint8_t getChannel() const;
        uint64_t getAddress() const;
        uint8_t getDatarate() const;
        bool isBroadcast() const;

    private: 
        void resetConnectionStats();
    private: 
        uint8_t m_channel;
        uint64_t m_address;
        uint8_t m_datarate; 
        bool m_isBroadcast;

        uint8_t m_failedMessagesMaximum;
        uint8_t m_failedMessagesCount;

        uint32_t m_relaxationCountMs;
        uint32_t m_relaxationPeriodMs;

        uint32_t m_lastSuccessfullMessageTime;
        uint32_t m_lastSuccessfullMessageTimeout;

        uint32_t m_failedMessageRetryTimeout;

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

        bool removeLink(CrtpLinkIdentifier * link, std::vector<CrtpResponseCallback>& callbacks);

        bool getLinkIdentifier(CrtpLinkIdentifier * link, uint8_t channel, uint64_t address) const;

        bool getHighestPriorityLink(CrtpLinkIdentifier * link, CrtpPort * port) const;

        bool getRandomLink(CrtpLinkIdentifier * link) const;
        bool getRandomRelaxedNonBroadcastLink(CrtpLinkIdentifier * link) const;

        void relaxLinks(uint8_t ms);

        /**
        * In order to call link functions with link identifiers we need these wrappers
        */
        void linkAddPacket(CrtpLinkIdentifier * link, CrtpPacket * packet, CrtpResponseCallback callback);
        bool linkGetPacket(CrtpLinkIdentifier * link, CrtpPort port,  CrtpPacket * packet);
        void linkNotifySuccessfullNullpacket(CrtpLinkIdentifier * link_id);
        void linkNotifySuccessfullMessage(CrtpLinkIdentifier * link_id, CrtpPort port);
        bool linkNotifyFailedMessage(CrtpLinkIdentifier * link_id);
        bool linkReleasePacket(CrtpLinkIdentifier  * link_id, CrtpPacket * responsePacket, CrtpResponseCallback & callback);
        double linkGetLinkQuality(CrtpLinkIdentifier * link_id);
        void linkTense(CrtpLinkIdentifier * link_id);
        void linkRelaxMs(CrtpLinkIdentifier * link_id, uint8_t ms);

    private: 
        void copyLinkIdentifier(CrtpLinkIdentifier * from_link, CrtpLinkIdentifier * to_link) const;
        void linkToIdentifier(const CrtpLink * link, CrtpLinkIdentifier * link_id) const;
        bool linkFromIdentifier(CrtpLink ** link, CrtpLinkIdentifier * link_id);
        std::map<std::pair<uint8_t, uint64_t>, libcrtp::CrtpLink> m_links;

        mutable std::mutex m_linksMutex;
};

} // namespace libcrtp