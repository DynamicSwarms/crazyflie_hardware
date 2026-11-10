#include "crazyradio/radio_logger.hpp"
#include <stdexcept>

namespace libradio::crazyradio {
RadioLogger::RadioLogger(const std::filesystem::path & directory, uint8_t channel)
: m_log(directory / "radio.log"), m_lastFlush(std::chrono::steady_clock::now())
{
    if (!m_log) throw std::runtime_error("Cannot open radio log");
    writeLogHeader(m_log, "USB", channel);
    m_log << "# timestamp transfer ack retries rssi(dBm) invalid_settings usb_reply_malformed\n"
          << "# 1=yes 0=no -=unavailable/disabled\n";
    m_log.flush();
}
void RadioLogger::logTransfer(const Transfer & transfer, const InlineAck & ack, bool ackEnabled)
{
    writeLogPrefix(m_log, transfer);
    const bool valid = !ack.usb_reply_malformed && !ack.invalid_settings;
    m_log << ' ' << (valid && ackEnabled ? (ack.ack ? "1" : "0") : "-") << ' ';
    if (valid && ackEnabled) m_log << static_cast<int>(ack.retry);
    else m_log << '-';
    m_log << ' ';
    if (ack.rssi_valid && ackEnabled) m_log << ack.rssi_dbm;
    else m_log << '-';
    m_log << ' ';
    if (!ack.usb_reply_malformed) m_log << ack.invalid_settings;
    else m_log << '-';
    m_log << ' ' << ack.usb_reply_malformed << '\n';
    const auto now = std::chrono::steady_clock::now();
    if (now - m_lastFlush >= std::chrono::seconds(1)) {
        m_log.flush();
        m_lastFlush = now;
    }
}
} // namespace libradio::crazyradio
