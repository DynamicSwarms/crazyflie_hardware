#pragma once

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/logic/logic.hpp"
#include "crtp_cpp/packer/crtp_packer.hpp"
#include "crtp_cpp/packer/localization_packer.hpp"
#include <vector>
#include <map>
#include <cstring> // Add this line

/**
 * @brief Handle localization-related data communication with the Crazyflie
 */
class LocalizationLogic : public Logic {
public:
    /**
     * @brief Constructor for LocalizationLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     */
    LocalizationLogic(CrtpLink* crtp_link);

    /**
     * @brief Send the current Crazyflie X, Y, Z position. This is going to be
     * forwarded to the Crazyflie's position estimator.
     * @param pos The position [x, y, z] as a vector of floats.
     */
    void send_extpos(const std::vector<float>& pos);

    /**
     * @brief Send ultra-wide-band LPP packet to dest_id.
     * @param dest_id Destination ID.
     * @param data The data to be sent.
     */
    void send_short_lpp_packet(uint8_t dest_id, const std::vector<uint8_t>& data);

    /**
     * @brief Send emergency stop.
     */
    void send_emergency_stop();

    /**
     * @brief Send emergency stop watchdog.
     */
    void send_emergency_stop_watchdog();

    /**
     * @brief Send the current Crazyflie pose (position [x, y, z] and
     * attitude quaternion [qx, qy, qz, qw]). This is going to be forwarded
     * to the Crazyflie's position estimator.
     * @param pos The position [x, y, z] as a vector of floats.
     * @param quat The quaternion [qx, qy, qz, qw] as a vector of floats.
     */
    void send_extpose(const std::vector<float>& pos, const std::vector<float>& quat);

    /**
     * @brief Send geometry and calibration data to persistent memory subsystem.
     * @param geo_list List of geometry basestations.
     * @param calib_list List of calibration basestations.
     */
    void send_lh_persist_data_packet(const std::vector<int>& geo_list, const std::vector<int>& calib_list);

private:
    LocalizationPacker packer;
    std::map<std::string, std::vector<float>> _decode_lh_angle(const std::vector<uint8_t>& data);
    float fp16_to_float(uint16_t fp16);
};

float LocalizationLogic::fp16_to_float(uint16_t fp16){
    uint32_t s = (fp16 >> 15) & 0x00000001; // sign
    uint32_t e = (fp16 >> 10) & 0x0000001f; // exponent
    uint32_t f = fp16 & 0x000003ff;         // fraction

    if (e == 0) {
        if (f == 0) {
            uint32_t result = s << 31;
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        } else {
            while (!(f & 0x00000400)) {
                f <<= 1;
                e -= 1;
            }
            e += 1;
            f &= ~0x00000400;
        }
    } else if (e == 31) {
        if (f == 0) {
            uint32_t result = (s << 31) | 0x7f800000;
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        } else {
            uint32_t result = (s << 31) | 0x7f800000 | (f << 13);
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        }
    }

    e += 127 - 15;
    f <<= 13;
    uint32_t result = (s << 31) | (e << 23) | f;
    float float_result;
    std::memcpy(&float_result, &result, sizeof(float));
    return float_result;
}