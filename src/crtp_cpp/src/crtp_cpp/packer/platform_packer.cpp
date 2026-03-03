#include "crtp_cpp/packer/platform_packer.hpp"

#define PORT_PLATFORM 13

#define CHANNEL_PLATFORM 0
#define CHANNEL_VERSION 1
#define CHANNEL_APP 3

#define PLATFORM_COMMAND_SET_CONT_WAVE 0
#define PLATFORM_COMMAND_REQUEST_ARMING 1
#define PLATFORM_COMMAND_REQUEST_CRASH_RECOVERY 2
#define PLATFORM_COMMAND_USER_NOTIFICATION 3

#define VERSION_COMMAND_GET_PROTOCOL 0
#define VERSION_COMMAND_GET_FIRMWARE 1
#define VERSION_COMMAND_GET_DEVICE_TYPE 2

PlatformPacker::PlatformPacker() 
    : CrtpPacker(PORT_PLATFORM) {}

CrtpPacket
PlatformPacker::prepare_packet(uint8_t channel, const std::vector<uint8_t>& data) 
{
    return CrtpPacker::prepare_packet(channel, data);
}

CrtpRequest
PlatformPacker::set_cont_wave(bool enable) 
{
    CrtpRequest request;
    std::vector<uint8_t> data = {PLATFORM_COMMAND_SET_CONT_WAVE};
    request.packet = prepare_packet(CHANNEL_PLATFORM, data);
    request.expects_response = true;
    request.matching_bytes = 1;
    return request;
}

CrtpRequest
PlatformPacker::request_arming(bool requested_state) 
{
    CrtpRequest request;
    std::vector<uint8_t> data = {PLATFORM_COMMAND_REQUEST_ARMING, requested_state ? 1 : 0};
    request.packet = prepare_packet(CHANNEL_PLATFORM, data);
    request.expects_response = true;
    request.matching_bytes = 1;
    return request;
}

CrtpRequest
PlatformPacker::request_crash_recovery() 
{
    CrtpRequest request;
    std::vector<uint8_t> data = {PLATFORM_COMMAND_REQUEST_CRASH_RECOVERY};
    request.packet = prepare_packet(CHANNEL_PLATFORM, data);
    request.expects_response = true;
    request.matching_bytes = 1;
    return request;
}

CrtpRequest
PlatformPacker::user_notification(uint8_t notification_code) 
{
    CrtpRequest request;
    std::vector<uint8_t> data = {PLATFORM_COMMAND_USER_NOTIFICATION, notification_code};
    request.packet = prepare_packet(CHANNEL_PLATFORM, data);
    request.expects_response = true;
    request.matching_bytes = 1;
    return request;
}

CrtpRequest
PlatformPacker::get_protocol() 
{
    CrtpRequest request;
    std::vector<uint8_t> data = {VERSION_COMMAND_GET_PROTOCOL};
    request.packet = prepare_packet(CHANNEL_VERSION, data);
    request.expects_response = true;
    request.matching_bytes = 1;
    return request;
}

CrtpRequest
PlatformPacker::get_firmware() 
{
    CrtpRequest request;
    std::vector<uint8_t> data = {VERSION_COMMAND_GET_FIRMWARE};
    request.packet = prepare_packet(CHANNEL_VERSION, data);
    request.expects_response = true;
    request.matching_bytes = 1;
    return request;
}

CrtpRequest
PlatformPacker::get_device_type() 
{
    std::vector<uint8_t> data = {VERSION_COMMAND_GET_DEVICE_TYPE};
    CrtpRequest request;
    request.packet = prepare_packet(CHANNEL_VERSION, data);
    request.expects_response = true;
    request.matching_bytes = 1;
    return request;
}