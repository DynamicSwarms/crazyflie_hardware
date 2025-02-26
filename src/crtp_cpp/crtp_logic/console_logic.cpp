#include "crtp_logic/console_logic.hpp"

ConsoleLogic::ConsoleLogic(CrtpLink* crtp_link)
    : Logic(crtp_link),
      packer(ConsolePacker()) {}

void ConsoleLogic::send_consolepacket() {
    auto packet = packer.consolepacket();
    link->send_packet_no_response(&packet);
}