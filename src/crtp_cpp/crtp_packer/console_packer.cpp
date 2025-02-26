#include "crtp_packer/console_packer.hpp"

#define PORT_CONSOLE 0
#define CHANNEL_CONSOLE 0

ConsolePacker::ConsolePacker()
    : CrtpPacker(PORT_CONSOLE) {}

CrtpPacket ConsolePacker::prepare_packet(const std::vector<uint8_t>& data) {
    return CrtpPacker::prepare_packet(CHANNEL_CONSOLE, data);
}

CrtpPacket ConsolePacker::consolepacket() {
    std::vector<uint8_t> data; // Empty vector for struct.pack("<")
    return prepare_packet(data);
}