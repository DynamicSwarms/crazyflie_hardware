#include "crtp_logic/link_layer_logic.hpp"

LinkLayerLogic::LinkLayerLogic(CrtpLink* crtp_link)
    : Logic(crtp_link),
      packer(LinkLayerPacker()) {}

void LinkLayerLogic::send_nullpacket() {
    auto packet = packer.nullpacket();
    link->send_packet_no_response(&packet);
}

void LinkLayerLogic::platform_power_down() {
    auto packet = packer.platform_power_down();
    link->send_packet_no_response(&packet);
}

void LinkLayerLogic::reboot_to_bootloader() {
    auto packet = packer.reset_init();
    link->send_packet_no_response(&packet);

    packet = packer.reset_to_bootloader();
    link->send_packet_no_response(&packet);
}

void LinkLayerLogic::reboot_to_firmware() {
    auto packet = packer.reset_init();
    link->send_packet_no_response(&packet);

    packet = packer.reset_to_firmware();
    link->send_packet_no_response(&packet);
}
