#pragma once

#include "crtp_cpp/packer/crtp_packer.hpp"
#include <vector>
#include <cstdint>

class PlatformPacker : public CrtpPacker {
public:
    PlatformPacker();

    
    CrtpRequest set_cont_wave(bool enable);

    // Request arming: if requested_state -> try to arm, else try to disarm
    CrtpRequest request_arming(bool requested_state);
    CrtpRequest request_crash_recovery();
    CrtpRequest user_notification(uint8_t notification_code);

    CrtpRequest get_protocol();
    CrtpRequest get_firmware();
    CrtpRequest get_device_type();

protected:
    CrtpPacket prepare_packet(uint8_t channel, const std::vector<uint8_t>& data);
};
