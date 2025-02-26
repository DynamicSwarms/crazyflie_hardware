#pragma once

#include <stdint.h>
#include <vector>
#include <crtp_link/crtp_link.hpp>

class CrtpPacker {
public:
  CrtpPacker(int port);
  virtual ~CrtpPacker() = default;

  CrtpPacket prepare_packet(int channel, const std::vector<uint8_t>& data);

protected:
  int port;
};