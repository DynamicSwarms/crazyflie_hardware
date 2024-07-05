#pragma once


class CrtpLink
{
    public: 
        CrtpLink(
            uint8_t channel,
            uint64_t address,
            uint8_t datarate
        );

        /** 
        *   Sets radio to the link's specific channel, address and datarate 
        */
        void setRadio(libcrazyradio::Crazyradio * radio);
    
        /**
        *  Adds a to be sent out Packet to the link. 
        */
        void addPacket(CrtpPacket * packet);

        /**
        *  Gets a crazyflies response, which should be crossreferenced to an yet unacknowledged request which excpected a response. 
        *  Returns False if nobody listened for a response.
        */
        bool releasePacket();

    private: 
        uint8_t m_channel;
        uint64_t m_address;
        uint8_t datarate; 


};