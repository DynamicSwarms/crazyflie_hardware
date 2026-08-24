#pragma once

#include <string>

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/logic/logic.hpp"
#include "crtp_cpp/packer/platform_packer.hpp"

class PlatformLogic : public Logic {
public:
    PlatformLogic(std::shared_ptr<CrtpLink> crtp_link);

    bool set_cont_wave();

    // Request arming: if requested_state -> try to arm, else try to disarm
    // Returns true if state was successfully changed
    bool request_arming(bool requested_state);
    bool request_crash_recovery();
    void user_notification(uint8_t notification_code);

    bool get_protocol(int &protocol);
    bool get_firmware(std::string &firmware);
    bool get_device_type(std::string &device_type);

private: 
    PlatformPacker packer;
};