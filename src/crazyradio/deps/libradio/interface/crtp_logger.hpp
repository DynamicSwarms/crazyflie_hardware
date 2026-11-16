#pragma once

#include "interface/logging.hpp"
#include "libcrtp/CrtpLink.hpp"
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace libradio {

// Logging is single-threaded; file setup happens during construction.
class CrtpLogger
{
public:
    CrtpLogger(bool logToFile, bool logToSocket, uint8_t channel,
        const std::string & transport, const std::filesystem::path & logRoot);
    ~CrtpLogger();

    void log(const libcrtp::CrtpLinkIdentifier * link,
        const libcrtp::CrtpPacket * packet, const libcrtp::CrtpPacket * responsePacket,
        bool responseValid, const Transfer & transfer);
    const std::filesystem::path & directory() const { return m_directory; }

private:
    bool m_logEnabled;
    std::ofstream m_logStream;
    std::stringstream formatCrtpPacket(const libcrtp::CrtpPacket * packet);
    std::filesystem::path m_directory;
    std::chrono::steady_clock::time_point m_lastFlush{};
};

} // namespace libradio
