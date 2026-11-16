#pragma once

#include "crazyradio/InlineAck.hpp"
#include "interface/logging.hpp"
#include <filesystem>
#include <fstream>

namespace libradio::crazyradio {
class RadioLogger
{
public:
    RadioLogger(bool logToFile, const std::filesystem::path & directory, uint8_t channel);
    void log(const Transfer & transfer, const InlineAck & ack, bool ackEnabled);

private:
    bool m_logEnabled;
    std::ofstream m_log;
    std::chrono::steady_clock::time_point m_lastFlush;
};
} // namespace libradio::crazyradio
