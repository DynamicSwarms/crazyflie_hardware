#include "crazyradio/crtp_logger.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <iomanip>
#include <unistd.h>

namespace crazyradio {

CrtpLogger::CrtpLogger(bool logEnabled, uint8_t channel)
    : m_logEnabled(logEnabled)
{
    if (m_logEnabled) {
        const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto logDirectory = rclcpp::get_log_directory() / "crazyradio" /
            (std::to_string(stamp) + "-" + std::to_string(getpid()));
        std::filesystem::create_directories(logDirectory);
        const auto logFile = logDirectory /
            ("crazyradio_log_" + std::to_string(channel) + ".log");
        m_logStream.open(logFile);
        if (!m_logStream.is_open()) {
            throw std::runtime_error("Failed to open log file");
        }
        const auto statsFile = logDirectory /
            ("crazyradio_stats_" + std::to_string(channel) + ".csv");
        m_statsStream.open(statsFile);
        if (!m_statsStream.is_open()) throw std::runtime_error("Failed to open radio statistics log");
        m_statsStream << "timestamp_us,steady_us,radio_channel,address_hex,drone_id,datarate,"
            "ack_enabled,crtp_port,crtp_channel,tx_bytes,ack_received,retries,weak_signal,"
            "rssi_dbm,raw_rssi,invalid_settings,malformed,usb_declared_bytes,usb_received_bytes,"
            "ack_payload_bytes,transfer_us,tx_hex,ack_hex\n";
        m_statsStream.flush();
        m_lastFlush = std::chrono::steady_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("crazyradio.logger"),
            "Packet and radio statistics logs: %s", logDirectory.c_str());
    }
}

CrtpLogger::~CrtpLogger()
{
    if (m_logEnabled && m_logStream.is_open()) {
        m_logStream.close();
    }
}

void CrtpLogger::logRadioTransaction(
    uint8_t channel, uint64_t address, uint8_t datarate, bool ackEnabled,
    const uint8_t* data, uint32_t length,
    const libradio::crazyradio::InlineAck& ack,
    std::chrono::nanoseconds logTime)
{
    if (!m_logEnabled) return;
    const auto now = std::chrono::steady_clock::now();
    m_statsStream << std::chrono::duration_cast<std::chrono::microseconds>(logTime).count()
        << ',' << std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()
        << ',' << static_cast<int>(channel) << ',' << std::hex << std::setfill('0')
        << std::setw(10) << address << std::dec << ',' << (address & 0xFF)
        << ',' << static_cast<int>(datarate) << ',' << ackEnabled
        << ',' << (length ? data[0] >> 4 : 0)
        << ',' << (length ? data[0] & 0x03 : 0) << ',' << length
        << ',' << ack.ack << ',' << static_cast<int>(ack.retry)
        << ',' << ack.weak_signal << ',';
    if (ack.rssi_valid && ackEnabled) m_statsStream << ack.rssi_dbm;
    m_statsStream << ',' << static_cast<int>(ack.raw_rssi)
        << ',' << ack.invalid_settings << ',' << ack.malformed
        << ',' << static_cast<int>(ack.total_length) << ',' << ack.received_length
        << ',' << static_cast<int>(ack.data_length) << ',' << ack.transfer_us << ',';
    for (uint32_t i = 0; i < length; ++i)
        m_statsStream << std::hex << std::setw(2) << static_cast<int>(data[i]);
    m_statsStream << ',';
    for (uint8_t i = 0; i < ack.data_length; ++i)
        m_statsStream << std::hex << std::setw(2) << static_cast<int>(ack.data[i]);
    m_statsStream << std::dec << '\n';
    // Bound crash loss without forcing a disk flush for every transaction.
    if (ack.malformed || now - m_lastFlush >= std::chrono::seconds(1)) {
        m_statsStream.flush();
        m_lastFlush = now;
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
