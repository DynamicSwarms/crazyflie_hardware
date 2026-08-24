#include "crtp_cpp/logic/platform_logic.hpp"

PlatformLogic::PlatformLogic(std::shared_ptr<CrtpLink> crtp_link)
    : Logic(crtp_link),
      packer(PlatformPacker()) {}

bool PlatformLogic::set_cont_wave() {
    CrtpRequest request = packer.set_cont_wave(true);
    auto response = link->send_packet(request);
    if (response.has_value())
    {
        return true;
    } 
    return false;
}

bool 
PlatformLogic::request_arming(bool requested_state) {
    CrtpRequest request = packer.request_arming(requested_state);
    auto response = link->send_packet(request);
    /**
     * data[0] = success;
     * data[1] = supervisorIsArmed();
     * (Shifted because first byte is command.)
     */
    if (response.has_value())
    {
        if (response->data[1] && (bool)response->data[2] == requested_state)
        {
            return true;
        }
    } 
    return false;
}

bool 
PlatformLogic::request_crash_recovery() {
    CrtpRequest request = packer.request_crash_recovery();
    auto response = link->send_packet(request);
    /**
     * data[0] = success;
     * data[1] = !supervisorIsCrashed();
     * (Shifted because first byte is command.)
    */
    if (response.has_value())
    {
        if (response->data[1] && response->data[2] == 1)
        {
            return true;
        }
    } 
    return false;
}

void 
PlatformLogic::user_notification(uint8_t notification_code) {
    CrtpRequest request = packer.user_notification(notification_code);
    link->send_packet(request);
}

bool 
PlatformLogic::get_protocol(
    int &protocol)
{
    CrtpRequest request = packer.get_protocol();
    auto response = link->send_packet(request);
    if (response.has_value()) {
        protocol = response->data[1] | 
               (response->data[2] << 8) | 
               (response->data[3] << 16) | 
               (response->data[4] << 24);
        return true;   
    }
    return false;
}

bool 
PlatformLogic::get_firmware(
    std::string &firmware)
{    
    CrtpRequest request = packer.get_firmware();
    auto response = link->send_packet(request);
    if (response.has_value()) {
        firmware.clear();
        for (int i = 1; i < response->data_length; i++) {
            firmware.push_back(static_cast<char>(response->data[i]));
        }
        return true;   
    }
    return false;
}

bool 
PlatformLogic::get_device_type(
    std::string &device_type)
{
    CrtpRequest request = packer.get_device_type();
    auto response = link->send_packet(request);
    if (response.has_value()) {
        device_type.clear();
        for (int i = 1; i < response->data_length; i++) {
            device_type.push_back(static_cast<char>(response->data[i]));
        }
        return true;   
    }
    return false;   
}

