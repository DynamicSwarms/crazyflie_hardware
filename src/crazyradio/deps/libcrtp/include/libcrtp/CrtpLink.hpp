#pragma once
#include <stdint.h>
#include <map>
#include <optional>
#include <random>

#include "libcrtp/CrtpPacketQueue.hpp"
#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/TickTimer.hpp"

namespace libcrtp {

/**
 * Failed transmissions use bounded randomized exponential backoff inspired
 * by Ethernet binary exponential backoff. It is adapted to this centrally
 * scheduled radio by using a non-zero 10 ms minimum; the random retry window
 * grows exponentially to a maximum of 200 ms.
 *
 * Background:
 * Goodman et al., "Stability of Binary Exponential Backoff"
 * https://doi.org/10.1145/44483.44488
 *
 * This Ethernet-inspired policy is intentionally simple. If dynamic fleet
 * scaling or retry load becomes limiting, Re-Backoff may be worth evaluating:
 * Bender et al., "How to Scale Exponential Backoff"
 * https://arxiv.org/abs/1402.5207
 */

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
         * Returns the current unresolved outbound packet, the highest priority
         * queued packet, or a null packet. The packet remains current until a
         * successful-send notification is received.
         */
        void getOutboundPacket(CrtpPacket * packet, bool * isPortPacket);

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
        void notifySuccessfullNullpacket(bool responseIsNullpacket);

        /**
         * Will remove the message from the Port because it was successfully sent out. 
        */
        void notifySuccessfullPortMessage(CrtpPort port, bool responseIsNullpacket);

        /** 
         * Notifies about a failed nullpacket attempt, returns true if link shall die.
         */
        bool notifyFailedNullpacket();

        /**
         * Notifies about a failed send attempt, returns true if link shall die
        */
        bool notifyFailedPortMessage();

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
        bool onFailedMessage();
        void resetBackoffTimer();
        
    private:
        std::map<CrtpPort, CrtpPacketQueue> m_crtpPortQueues;

        uint8_t m_channel;
        uint64_t m_address;
        uint8_t m_datarate; 
        bool m_isBroadcast;
    
    // Configurable parameters
    private: 
        uint8_t m_failedMessagesMaximum;
        uint32_t m_nullpacketRelaxationMs;
        uint32_t m_lastSuccessfullMessageTimeoutMs;
        uint32_t m_minimumBackoffMs;
        uint32_t m_maximumBackoffMs;
    
    // Internal state
    private: 
        uint8_t m_failedMessagesCount;
        TickTimer m_backoffTimer;
        TickTimer m_relaxationTimer;
        TickTimer m_livenessTimer;
        std::mt19937 m_randomGenerator;
        uint64_t m_linkQuality; // 64 bits of failed and successful messages (bits)

        struct CurrentOutbound
        {
            CrtpPacket packet;
            bool isPortPacket;
        };
        std::optional<CurrentOutbound> m_currentOutbound;
};

} // namespace libcrtp
