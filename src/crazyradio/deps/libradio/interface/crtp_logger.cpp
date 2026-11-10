#include "interface/crtp_logger.hpp"

#include <ctime>
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <stdexcept>

namespace libradio {
void writeLogPrefix(std::ostream & out, const Transfer & transfer)
{
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        transfer.completed.time_since_epoch()).count();
    out << '[' << ns / 1000000000 << '.' << std::setfill('0') << std::setw(9)
        << ns % 1000000000 << "] " << std::setfill(' ') << std::setw(10) << transfer.id;
}
void writeLogHeader(std::ostream & out, const std::string & transport, uint8_t channel)
{
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm local{};
    localtime_r(&now, &local);
    out << "# " << std::put_time(&local, "%Y-%m-%d | %Z (%z)")
        << " | " << transport << " | channel " << static_cast<int>(channel) << '\n';
}
CrtpLogger::CrtpLogger(bool logToFile, bool logToSocket, uint8_t channel,
    const std::string & transport, const std::filesystem::path & logRoot)
    : m_logEnabled(logToFile)
{
    if (logToSocket) throw std::invalid_argument("Packet socket logging is not implemented yet");
    if (m_logEnabled) {
        const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto logDirectory = logRoot / "crazyradio" /
            (std::to_string(stamp) + "-" + std::to_string(getpid()) + "-" + std::to_string(channel));
        std::filesystem::create_directories(logDirectory);
        const auto logFile = logDirectory / "packets.log";
        m_logStream.open(logFile);
        if (!m_logStream.is_open()) {
            throw std::runtime_error("Failed to open log file");
        }
        m_directory = logDirectory;
        writeLogHeader(m_logStream, transport, channel);
        m_logStream << "# timestamp transfer address [port:channel] hex bytes; next line = response\n";
        m_logStream.flush();
        m_lastFlush = std::chrono::steady_clock::now();
        std::clog << "Radio logs: " << logDirectory << '\n';
    }
}

CrtpLogger::~CrtpLogger()
{
    if (m_logEnabled && m_logStream.is_open()) {
        m_logStream.close();
    }
}

Transfer CrtpLogger::logCommunication(const libcrtp::CrtpLinkIdentifier * link,
    const libcrtp::CrtpPacket * packet, const libcrtp::CrtpPacket * responsePacket,
    bool responseValid, Transfer transfer)
{
    if (m_logEnabled)
    {
        if (!transfer.id) transfer = {++m_transferId, std::chrono::system_clock::now()};
        std::stringstream ss;
        writeLogPrefix(ss, transfer);
        ss << ' ' << std::hex << std::setfill('0') << std::setw(10) << link->address;

        m_logStream << ss.str() << formatCrtpPacket(packet).str() << '\n';
        if (responseValid) m_logStream << '\t' << formatCrtpPacket(responsePacket).str();
        m_logStream << '\n';

        const auto now = std::chrono::steady_clock::now();
        if (now - m_lastFlush >= std::chrono::seconds(1)) {
            m_logStream.flush();
            m_lastFlush = now;
        }
    }
    return transfer;
}

std::stringstream CrtpLogger::formatCrtpPacket(const libcrtp::CrtpPacket * packet)
{
    std::stringstream ss;
    ss << std::dec << " [" << (int)packet->port << ":" << (int)packet->channel << "] ";
    for (int i = 0; i < packet->dataLength; i++)
    {
        ss << std::hex << (int)packet->data[i] << " ";
    }
    return ss;
}

} // namespace libradio
