#include "crtp_cpp/logic/console_logic.hpp"

ConsoleLogic::ConsoleLogic(CrtpLink* crtp_link)
    : Logic(crtp_link),
      packer(ConsolePacker()) {}

void ConsoleLogic::send_consolepacket() {
    CrtpRequest request;
    request.packet = packer.consolepacket();
    link->send_packet_no_response(request);
}