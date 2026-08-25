#pragma once

#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"

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
    private:
        bool m_logEnabled;
        std::ofstream m_logStream;

        std::stringstream formatCrtpPacket(libcrtp::CrtpPacket *packet);
};

} // namespace crazyradio
