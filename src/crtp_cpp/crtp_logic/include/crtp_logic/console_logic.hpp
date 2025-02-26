#pragma once

#include "crtp_link/crtp_link.hpp"
#include "crtp_logic/logic.hpp"
#include "crtp_packer/console_packer.hpp"

/**
 * @brief Logic for sending console packets.
 */
class ConsoleLogic : public Logic {
public:
    /**
     * @brief Constructor for ConsoleLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     */
    ConsoleLogic(CrtpLink* crtp_link);

    /**
     * @brief Sends a console packet.
     */
    void send_consolepacket();

private:
    ConsolePacker packer;
};
