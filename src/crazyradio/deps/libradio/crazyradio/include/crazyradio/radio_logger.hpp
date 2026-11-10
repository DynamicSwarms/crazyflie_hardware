#pragma once

#include "crazyradio/InlineAck.hpp"
#include "interface/logging.hpp"
#include <filesystem>
#include <fstream>

namespace libradio::crazyradio {
class RadioLogger
{
public:
    RadioLogger(const std::filesystem::path & directory, uint8_t channel);
    void logTransfer(const Transfer & transfer, const InlineAck & ack, bool ackEnabled);

private:
    std::ofstream m_log;
    std::chrono::steady_clock::time_point m_lastFlush;
};
} // namespace libradio::crazyradio
