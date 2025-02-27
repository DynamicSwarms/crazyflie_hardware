
#include "crtp_cpp/packer/parameters_packer.hpp"
#include <cstring>

#define PORT_PARAMETER 0x02

#define TOC_CHANNEL 0
#define READ_CHANNEL 1
#define WRITE_CHANNEL 2
#define MISC_CHANNEL 3

ParametersPacker::ParametersPacker()
    : TocPacker(PORT_PARAMETER) {}

CrtpPacket ParametersPacker::prepare_packet(const std::vector<uint8_t>& data, uint8_t channel) {
    return CrtpPacker::prepare_packet(channel, data);
}

CrtpRequest ParametersPacker::set_parameter(uint16_t id, ParamValue value, ParamType type) {
    CrtpRequest request;
    std::vector<uint8_t> data(2); // ID (uint16_t)
    // TODO: To Be implemented
    request.packet = prepare_packet(data, WRITE_CHANNEL);
    return request;
}
