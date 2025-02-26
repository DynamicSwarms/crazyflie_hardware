#include "crtp_logic/basic_commander_logic.hpp"

BasicCommanderLogic::BasicCommanderLogic(CrtpLink* crtp_link)
    : Logic(crtp_link),
      packer(BasicCommanderPacker()) {
    _x_mode = false; // Initialize x-mode to false by default
}

void BasicCommanderLogic::set_client_xmode(bool enabled) {
    _x_mode = enabled;
}

void BasicCommanderLogic::send_setpoint(float roll, float pitch, float yawrate, uint16_t thrust) {
    if (_x_mode) {
        float temp_roll = 0.707f * (roll - pitch);
        float temp_pitch = 0.707f * (roll + pitch);
        roll = temp_roll;
        pitch = temp_pitch;
    }

    auto packet = packer.send_setpoint(roll, -pitch, yawrate, thrust);
    link->send_packet_no_response(&packet);
}
