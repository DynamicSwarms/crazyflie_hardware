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

        /**
         * Gets a packet from the port, if available.
         * Returns true if a packet was found, false otherwise.
         * The packet is not removed from the port, call notifySuccessfullPortMessage to do so.
         */
        bool getPacket(CrtpPort port, CrtpPacket * packet);

        /**
         * Returns the port with highest priority with a packet to send inside.
         * Returns CrtpPort::NO_PORT if completely empty
         */
        CrtpPort getPriorityPort() const;
        
        /**
        *  Pass a response packet, will be crossreferenced to an yet unacknowledged request which excpected a response. 
        *  Returns False if nobody listened for a response.
        */
        bool releasePacket(CrtpPacket * packet, CrtpResponseCallback & callback);      
        
        /**
         * A nullpacket from polling was successfully sent. 
         * Reset connection stats.
        */
        void notifySuccessfullNullpacket();

        /**
         * Will remove the message from the Port because it was successfully sent out. 
        */
        void notifySuccessfullPortMessage(CrtpPort port);

        /**
         * Notifies about a failed send attempt, returns true if link shall die
        */
        bool notifyFailedMessage();

        /**
         * This is called before decontrstuction.
         * This way the callbacks can be returned with false
        */
        void retrieveAllCallbacks(std::vector<CrtpResponseCallback>& callbacks);

        // Check if the link is relaxed and nullpacket can be sent
        bool isRelaxed() const;

        /**
         * Give time in ms to the link, so it can update its internal state.
         */
        void tickMs(uint8_t ms);

        /**
         * Returns the link quality as a double between 0 and 1.
         * 0 means no messages were sent, 1 means all messages were sent successfully.
         * Average over the last 64 messages.
         */
        double getLinkQuality() const;

        uint8_t getChannel() const;
        uint64_t getAddress() const;
        uint8_t getDatarate() const;
        bool isBroadcast() const;

    private: 
        void onSuccessfullMessage();
        
    // Configurable parameters
    private: 
        uint8_t m_failedMessagesMaximum;
        uint32_t m_relaxationPeriodMs;
        uint32_t m_lastSuccessfullMessageTimeout;
        uint32_t m_failedMessageRetryTimeout;

    private: 
        uint8_t m_failedMessagesCount;
        uint32_t m_relaxationCountMs;
        uint32_t m_lastSuccessfullMessageTime;
        
        std::map<CrtpPort, CrtpPacketQueue> m_crtpPortQueues;

        uint64_t m_linkQuality; // 64 bits of failed and successful messages (bits)

        uint8_t m_channel;
        uint64_t m_address;
        uint8_t m_datarate; 
        bool m_isBroadcast;
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

        /**
         * Removes a link from the container.
         * Returns true if the link was removed, false if it was not found.
         * The callbacks are returned to the caller, they should probably be called with a nullpacket.
         */
        bool removeLink(CrtpLinkIdentifier * link, std::vector<CrtpResponseCallback>& callbacks);

        /**
         * Returns true if the link was found and copied to the link identifier.
         * If the link is not found, false is returned.
         */
        bool getLinkIdentifier(CrtpLinkIdentifier * link, uint8_t channel, uint64_t address) const;

        /**
         * Returns true if a link with the highest priority port is found.
         * The link identifier is copied to the link parameter and the port is set to the highest priority port of the link.
         */
        bool getHighestPriorityLink(CrtpLinkIdentifier * link, CrtpPort * port) const;

        /**
         * Returns a random relaxed link which is not a broadcast link.
         * If no such link is found, false is returned.
         */
        bool getRandomRelaxedNonBroadcastLink(CrtpLinkIdentifier * link) const;

        /**
         * Get connection statistics for all links.
         */
        void getConnectionStats(std::vector<CrtpLinkIdentifier>& links, std::vector<double>& quality) const;

        /**
        * In order to call link functions with link identifiers we need these wrappers
        */
        void tickLinksMs(uint8_t ms);
        
        void linkAddPacket(CrtpLinkIdentifier * link, CrtpPacket * packet, CrtpResponseCallback callback);
        bool linkGetHighestPriorityPacket(CrtpLinkIdentifier * link, CrtpPacket * packet);
        bool linkReleasePacket(CrtpLinkIdentifier  * link_id, CrtpPacket * responsePacket, CrtpResponseCallback & callback);

        void linkNotifySuccessfullNullpacket(CrtpLinkIdentifier * link_id);
        void linkNotifySuccessfullPortMessage(CrtpLinkIdentifier * link_id, CrtpPort port);
        bool linkNotifyFailedMessage(CrtpLinkIdentifier * link_id);

    private: 
        void copyLinkIdentifier(CrtpLinkIdentifier * from_link, CrtpLinkIdentifier * to_link) const;
        void linkToIdentifier(const CrtpLink * link, CrtpLinkIdentifier * link_id) const;
        bool linkFromIdentifier(CrtpLink ** link, CrtpLinkIdentifier * link_id);
        std::map<std::pair<uint8_t, uint64_t>, libcrtp::CrtpLink> m_links;

        mutable std::mutex m_linksMutex;
};

} // namespace libcrtp