#pragma once

#include "crtp_packer/crtp_packer.hpp"
#include <vector>
#include <cstdint>

class ConsolePacker : public CrtpPacker {
public:
    ConsolePacker();

    CrtpPacket consolepacket();

protected:
    CrtpPacket prepare_packet(const std::vector<uint8_t>& data);
};