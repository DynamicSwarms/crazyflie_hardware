#include "crtp_cpp/logic/console_logic.hpp"

#include <utility>

ConsoleLogic::ConsoleLogic(std::shared_ptr<CrtpLink>crtp_link)
    : Logic(crtp_link),
      packer(ConsolePacker())
{
    link->add_callback(PORT_CONSOLE,
        std::bind(&ConsoleLogic::crtp_response_callback, this, std::placeholders::_1));
}

void ConsoleLogic::send_consolepacket()
{
    CrtpRequest request;
    request.packet = packer.consolepacket();
    link->send_packet_no_response(request);
}

void ConsoleLogic::set_message_callback(MessageCallback callback)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_callback = std::move(callback);
}

void ConsoleLogic::crtp_response_callback(const CrtpPacket &packet)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (packet.channel != CHANNEL_CONSOLE || packet.data_length > sizeof(packet.data)) return;

    m_message.append(reinterpret_cast<const char *>(packet.data), packet.data_length);

    size_t end;
    while ((end = m_message.find('\n')) != std::string::npos)
    {
        std::string line = m_message.substr(0, end);
        m_message.erase(0, end + 1);

        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (m_callback) m_callback(line);
    }
}
