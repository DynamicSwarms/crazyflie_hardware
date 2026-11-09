#pragma once

#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"
#include "crazyradio/InlineAck.hpp"

#include <fstream>
#include <cstdint>
#include <chrono>
#include <sstream>

namespace crazyradio {

class CrtpLogger
{
    public:
        CrtpLogger(bool logEnabled, uint8_t channel);

        virtual ~CrtpLogger();

        void logCommunication(
            libcrtp::CrtpLinkIdentifier *link,
            libcrtp::CrtpPacket *packet,
            libcrtp::CrtpPacket *responsePacket,
            bool responseValid,
            std::chrono::nanoseconds logTime);

        void logRadioTransaction(
            uint8_t channel, uint64_t address, uint8_t datarate, bool ackEnabled,
            const uint8_t* data, uint32_t length,
            const libradio::crazyradio::InlineAck& ack,
            std::chrono::nanoseconds logTime);
    private:
        bool m_logEnabled;
        std::ofstream m_logStream;
        std::ofstream m_statsStream;
        std::chrono::steady_clock::time_point m_lastFlush{};

        std::stringstream formatCrtpPacket(libcrtp::CrtpPacket *packet);
};

} // namespace crazyradio
