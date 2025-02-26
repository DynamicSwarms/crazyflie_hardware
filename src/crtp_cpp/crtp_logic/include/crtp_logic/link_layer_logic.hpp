#pragma once

#include "crtp_link/crtp_link.hpp"
#include "crtp_logic/logic.hpp"
#include "crtp_packer/crtp_packer.hpp"
#include "crtp_packer/link_layer_packer.hpp"

/**
 * @brief Logic for Link Layer commands.
 */
class LinkLayerLogic : public Logic {
public:
    /**
     * @brief Constructor for LinkLayerLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     */
    LinkLayerLogic(CrtpLink* crtp_link);

    /**
     * @brief Sends a null packet.
     */
    void send_nullpacket();

    /**
     * @brief Sends a platform power down command.
     */
    void platform_power_down();

    /**
     * @brief Reboots the Crazyflie to the bootloader.
     */
    void reboot_to_bootloader();

    /**
     * @brief Reboots the Crazyflie to the firmware.
     */
    void reboot_to_firmware();

private:
    LinkLayerPacker packer;
};