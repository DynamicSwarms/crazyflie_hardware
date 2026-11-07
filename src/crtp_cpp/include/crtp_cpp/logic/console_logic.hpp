#pragma once

#include <mutex>
#include <string>

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/logic/logic.hpp"
#include "crtp_cpp/packer/console_packer.hpp"

/**
 * @brief Logic for sending console packets and assembling received messages.
 * The owner must stop packet dispatch before destroying this object.
 */
class ConsoleLogic : public Logic
{
public:
    /**
     * @brief Constructor for ConsoleLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     */
    using MessageCallback = std::function<void(const std::string &)>;

    ConsoleLogic(std::shared_ptr<CrtpLink>crtp_link);
    void set_message_callback(MessageCallback callback);

    /**
     * @brief Sends a console packet.
     */
    void send_consolepacket();

private:
    void crtp_response_callback(const CrtpPacket &packet);

private:
    ConsolePacker packer;
    std::mutex m_mutex;
    std::string m_message;
    MessageCallback m_callback;
};
