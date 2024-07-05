#pragma once


class CrtpPacket
{
    public: 
        CrtpPacket(
            uint8_t channel,
            uint8_t port,
            uint8_t[31] data,
            uint8_t data_length
        );

        uint8_t getChannel();
        uint8_t getPort();
        uint8_t * getData();
        uint8_t getDataLength();

    private: 
        uint8_t m_channel;
        uint8_t m_port;
        uint8_t[31] m_data;
        uint8_t m_data_length;
};