#include "crazyradio/crtp_logger.hpp"

#include "rclcpp/logger.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

namespace crazyradio {

CrtpLogger::CrtpLogger(bool logEnabled, uint8_t channel)
    : m_logEnabled(logEnabled)
{
    if (m_logEnabled) {
        const auto logDirectory = rclcpp::get_log_directory() / "crazyradio";
        std::filesystem::create_directories(logDirectory);
        const auto logFile = logDirectory /
            ("crazyradio_log_" + std::to_string(channel) + ".log");
        m_logStream.open(logFile);
        if (!m_logStream.is_open()) {
            throw std::runtime_error("Failed to open log file");
        }
    }
}

CrtpLogger::~CrtpLogger()
{
    if (m_logEnabled && m_logStream.is_open()) {
        m_logStream.close();
    }
}

void CrtpLogger::logCommunication(
        libcrtp::CrtpLinkIdentifier *link,
        libcrtp::CrtpPacket *packet,
        libcrtp::CrtpPacket *responsePacket,
        bool responseValid,
        std::chrono::nanoseconds logTime)
    {
        if (m_logEnabled)
        {
            std::stringstream ss;
            auto micros = std::chrono::duration_cast<std::chrono::microseconds>(logTime).count();
            ss << "[" << (long int)(micros) << "] ";
            ss << std::hex << (int)(uint8_t)(link->address & 0xFF);

            m_logStream << ss.str() << formatCrtpPacket(packet).str() << std::endl;
            if (responseValid) m_logStream << '\t' << formatCrtpPacket(responsePacket).str();
            m_logStream << std::endl;
        }
    }

std::stringstream CrtpLogger::formatCrtpPacket(libcrtp::CrtpPacket *packet)
{
    std::stringstream ss;
    ss << std::dec << " [" << (int)packet->port << ":" << (int)packet->channel << "] ";
    for (int i = 0; i < packet->dataLength; i++)
    {
        ss << std::hex << (int)packet->data[i] << " ";
    }
    return ss;
}

} // namespace crazyradio
