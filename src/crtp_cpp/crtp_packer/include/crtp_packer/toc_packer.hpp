#pragma once

#include "crtp_packer.hpp"
#include <memory>
#include <functional>
#include <vector>
#include <cstdint>

class TocPacker : public CrtpPacker {
public:
    TocPacker(int port);

    std::tuple<CrtpPacket, bool, int> get_toc_info();
    std::tuple<CrtpPacket, bool, int> get_toc_item(uint16_t index);

protected:
    CrtpPacket prepare_packet(const std::vector<uint8_t>& data);
};
