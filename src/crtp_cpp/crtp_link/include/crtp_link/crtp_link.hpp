#pragma once

#include <stdint.h>
#include <vector>
#include <tuple>

struct CrtpPacket
{
    uint8_t port;
    uint8_t channel;
    uint8_t data[31];
    uint8_t dataLength;

    bool expectsResponse;
    uint8_t matchingBytes;
    bool obeysOrdering;
};

class CrtpLink {
public:
  CrtpLink(int channel, std::tuple<int> address, int datarate);

  virtual ~CrtpLink() = default; // Important: Add a virtual destructor

  virtual void send_packet_no_response(CrtpPacket * packet, bool expects_response = false, int matching_bytes = 0);
  virtual CrtpPacket send_packet(CrtpPacket * packet, bool expects_response, int matching_bytes) ;
  virtual std::vector<CrtpPacket> send_batch_request(const std::vector<CrtpPacket>);

protected:
  int channel;
  std::tuple<int> address;
  int datarate;
};