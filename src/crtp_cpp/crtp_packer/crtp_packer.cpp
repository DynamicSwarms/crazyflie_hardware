#include "crtp_packer/crtp_packer.hpp"

CrtpPacker::CrtpPacker(int port) : port(port) {}

CrtpPacket CrtpPacker::prepare_packet(int channel, const std::vector<uint8_t>& data)
{
    CrtpPacket packet; 
    packet.port = this->port;
    packet.channel = channel;
    copy(data.begin(), data.end(), packet.data);

    return packet;
}