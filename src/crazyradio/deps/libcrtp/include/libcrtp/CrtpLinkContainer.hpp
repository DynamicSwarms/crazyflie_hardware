#pragma once
#include <stdint.h>
#include <map>
#include <mutex>
#include <random>

#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"
#include "libcrtp/CrtpContainerStats.hpp"

namespace libcrtp {


class CrtpLinkContainer
{
    public:
        CrtpLinkContainer();
        virtual ~CrtpLinkContainer();

        /**
         * Adds Link to Container, if already present will not overwrite
         */
        void addLink(uint8_t channel, uint64_t address, uint8_t datarate);

        /** Stop creating new links. Existing links can still be used and drained. */
        void stopLinkCreation();

        /**
         * Removes a link from the container.
         * Returns true if the link was removed, false if it was not found.
         * The callbacks are returned to the caller, they should probably be called with a nullpacket.
         */
        bool removeLink(CrtpLinkIdentifier * link, std::vector<CrtpResponseCallback>& callbacks);

        /** Removes all links atomically and returns their identifiers and pending callbacks. */
        void removeAllLinks(std::vector<CrtpLinkIdentifier>& links,
                            std::vector<CrtpResponseCallback>& callbacks);

        /**
         * Returns true if the link was found and copied to the link identifier.
         * If the link is not found, false is returned.
         */
        bool getLinkIdentifier(CrtpLinkIdentifier * link, uint8_t channel, uint64_t address) const;

        /**
         * Returns true if a link with the highest priority port is found.
         * The link identifier is copied to the link parameter and the port is set to the highest priority port of the link.
         * Links with equal CRTP port priority are selected randomly to avoid capture
         * caused by deterministic map iteration order.
         */
        bool getHighestPriorityLink(CrtpLinkIdentifier * link, CrtpPort * port) const;

        /**
         * Returns a random relaxed link which is not a broadcast link.
         * If no such link is found, false is returned.
         */
        bool getRandomRelaxedNonBroadcastLink(CrtpLinkIdentifier * link) const;

        /** Return an aggregate statistics snapshot under the container lock. */
        CrtpContainerStats getStats();

        /** Get connection statistics for all links. */
        void getConnectionStats(std::vector<CrtpLinkIdentifier>& links, std::vector<double>& quality) const;

        /**
        * In order to call link functions with link identifiers we need these wrappers
        */
        void tickLinksMs(uint8_t ms);
        
        /** Returns false if the link no longer exists. */
        bool linkAddPacket(CrtpLinkIdentifier * link, CrtpPacket * packet, CrtpResponseCallback callback);
        bool linkGetOutboundPacket(CrtpLinkIdentifier * link, CrtpPacket * packet, bool * isPortPacket);
        bool linkReleasePacket(CrtpLinkIdentifier  * link_id, CrtpPacket * responsePacket, CrtpResponseCallback & callback);

        void linkNotifySuccessfullNullpacket(CrtpLinkIdentifier * link_id, bool responseIsNullpacket);
        void linkNotifySuccessfullPortMessage(CrtpLinkIdentifier * link_id, CrtpPort port, bool responseIsNullpacket);
        bool linkNotifyFailedNullpacket(CrtpLinkIdentifier * link_id);
        bool linkNotifyFailedPortMessage(CrtpLinkIdentifier * link_id);

    private: 
        void copyLinkIdentifier(CrtpLinkIdentifier * from_link, CrtpLinkIdentifier * to_link) const;
        void linkToIdentifier(const CrtpLink * link, CrtpLinkIdentifier * link_id) const;
        bool linkFromIdentifier(CrtpLink ** link, CrtpLinkIdentifier * link_id);
        size_t randomIndex(size_t size) const;
        std::map<std::pair<uint8_t, uint64_t>, libcrtp::CrtpLink> m_links;
        bool m_acceptingLinks{true};
        uint64_t m_enqueuedPackets{0}; // Protected by m_linksMutex; never reset on link removal.

        mutable std::mutex m_linksMutex;
        mutable std::mt19937 m_randomGenerator;
};

} // namespace libcrtp
