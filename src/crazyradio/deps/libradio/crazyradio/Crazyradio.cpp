#include "crazyradio/Crazyradio.hpp"


#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cstring>

#include <libusb-1.0/libusb.h>
namespace libradio::crazyradio {
enum
{
    SET_RADIO_CHANNEL   = 0x01,
    SET_RADIO_ADDRESS   = 0x02,
    SET_DATA_RATE       = 0x03,
    SET_RADIO_POWER     = 0x04,
    SET_RADIO_ARD       = 0x05,
    SET_RADIO_ARC       = 0x06,
    ACK_ENABLE          = 0x10,
    SET_CONT_CARRIER    = 0x20,
    SCANN_CHANNELS      = 0x21,
    SET_INLINE_MODE     = 0x23,
    LAUNCH_BOOTLOADER   = 0xFF,
};

Crazyradio::Crazyradio() 
    : USBDevice(0x1915, 0x7777)
{
    bool success = false;
    std::vector<std::string> errors; // Store errors for potential later use.

    for (int deviceId = 0; deviceId < 5; deviceId++) {
        try {
            open(deviceId);
            success = true; // Open succeeded, so break the loop
            break;
        } catch (const std::runtime_error& e) {
            errors.push_back(e.what()); // Store the error message
        }
    }
    if (!success) {
        //If open did not succeed, throw an error.
        std::string combinedError = "Failed to open any device: ";
        for(const auto& error : errors) {
            combinedError += error + "; ";
        }
        throw std::runtime_error(combinedError);
    }
    
    setContCarrier(false);
    setPower(Power_0DBM);
    setArc(3);
    setArdBytes(32);
    
    const auto version = this->version();
    const bool is_supported_crazyradio2 = (version.first >= 5 && version.second >= 1) && // Crazyradio 2.0 with first working firmware
                                          (version.first != 0x99); // but not a Crazyradio PA
    if (!is_supported_crazyradio2) {
        throw std::runtime_error("Unsupported Crazyradio version: " + std::to_string(version.first) + "." + std::to_string(version.second) 
        + "Please use a Crazyradio 2.0 with up to date firmware.");
    }

    setInlineMode(true); 

    std::cerr << "Crazyradio USB initialized with Inline Mode enabled." << std::endl;
}

Crazyradio::~Crazyradio()
{   
    std::cerr << "Crazyradio USB stopped." << std::endl;
}



bool Crazyradio::sendCrtpPacket(
        const libcrtp::CrtpLinkIdentifier * link,
        const libcrtp::CrtpPacket * packet,
        libcrtp::CrtpPacket * responsePacket)
{   
    libradio::crazyradio::Crazyradio::Ack ack;

    uint8_t data[32];
    Datarate datarate;
    switch (link->datarate)
    {
        case 2: datarate = libradio::crazyradio::Crazyradio::Datarate_2MPS; break;
        case 1: datarate = libradio::crazyradio::Crazyradio::Datarate_1MPS; break;
        default: datarate = libradio::crazyradio::Crazyradio::Datarate_250KPS; break;
    }

    SafeLinkState * safeLink = nullptr;
    if (!link->isBroadcast) {
        auto& state = m_safeLinkStates[safeLinkKey(link)];
        if (!state.initialized) {
            state.enabled = enableSafeLink(link, datarate);
            state.initialized = true;
            state.up = 0;
            state.down = 0;
        }
        safeLink = &state;
    }

    data[0] = packet->port << 4 | (packet->channel & 0x03);
    if (safeLink && safeLink->enabled) {
        data[0] |= (safeLink->up << 3) | (safeLink->down << 2);
    }
    memcpy(&data[1], &packet->data, packet->dataLength);

    sendPacketInline(
        data,
        1 + packet->dataLength, 
        datarate,
        link->channel,
        link->address,
        ! link->isBroadcast,
        ack
    );
    
    if (link->isBroadcast) return true;    
    if (!ack.ack) {
        return false;
    }

    if (safeLink && safeLink->enabled) {
        safeLink->up ^= 1;
    }

    if (ack.total_length < 2) // If acked, there must be at least a nullpacket beeing sent back (channel/port set)
    {
        /* The Bug in https://github.com/bitcraze/crazyflie-firmware/issues/703 prevents a response from beeing sent back from the crazyflie.
            *  The message however gets succesfully received by the crazyflie.
            *  For now we just assume that a nullpacket would have been sent from crazyflie, in order not to break any other code.
            */
        std::cerr <<  "Empty response #703" << std::endl;
        memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
        return true;
    } 
    
    if (safeLink && safeLink->enabled) {
        if (ack.total_length > 2) {
            const uint8_t receivedDown = (ack.data[0] >> 2) & 0x01;
            if (receivedDown != safeLink->down) {
                memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
                std::cerr << "SafeLink: Down bit mismatch, expected " << (int)safeLink->down << " but got " << (int)receivedDown << std::endl;
                return true;
            }

            safeLink->down ^= 1;
            ack.data[0] &= 0xF3;
        }
    }

    ackToCrtpPacket(&ack, responsePacket);
    return true;
}

Crazyradio::SafeLinkKey Crazyradio::safeLinkKey(
    const libcrtp::CrtpLinkIdentifier * link) const
{
    return {link->channel, link->address, link->datarate};
}

bool Crazyradio::enableSafeLink(
    const libcrtp::CrtpLinkIdentifier * link,
    Datarate datarate)
{
    const uint8_t request[] = {0xFF, 0x05, 0x01};

    for (int attempt = 0; attempt < 10; ++attempt) {
        Ack ack;
        sendPacketInline(
            request,
            sizeof(request),
            datarate,
            link->channel,
            link->address,
            true,
            ack);

        if (ack.ack && ack.total_length == 5 &&
            ack.data[0] == 0xFF && ack.data[1] == 0x05 && ack.data[2] == 0x01) {
            return true;
        }
    }

    return false;
}

void Crazyradio::resetLink(const libcrtp::CrtpLinkIdentifier * link)
{
    std::cerr << "Link Reset for CF 0x" << std::hex << (int)(uint8_t)(link->address & 0xFF) << std::dec << std::endl;
    m_safeLinkStates.erase(safeLinkKey(link));
}

void Crazyradio::sendPacketInline(
    const uint8_t* data,
    uint32_t length, 
    Datarate datarate,
    uint8_t channel,
    uint64_t address,
    bool ackEnabled,
    Ack& result
)
{
    uint8_t inlineData[40];
    inlineData[0] = 8 + length;
    inlineData[1] = datarate | (ackEnabled << 4);
    inlineData[2] = channel;
    inlineData[3] = (address >> 32) & 0xFF;
    inlineData[4] = (address >> 24) & 0xFF;
    inlineData[5] = (address >> 16) & 0xFF;
    inlineData[6] = (address >> 8) & 0xFF;
    inlineData[7] = (address >> 0) & 0xFF;
    memcpy(&inlineData[8], data, length);

    sendPacket(inlineData, 8 + length, ackEnabled,result);
}

void Crazyradio::sendPacket(
    const uint8_t * data,
    uint32_t length,
    bool ackEnabled,
    Ack& result
)
{
    result.ack = false;

    int status, transferred;

    if (!m_handle) throw std::runtime_error("No valid device handle!");

    status = libusb_bulk_transfer(
        m_handle, 
        (0x01 | LIBUSB_ENDPOINT_OUT),
        (uint8_t *)data,
        length, 
        &transferred,
        /*timeout*/ 100);

    if (status != LIBUSB_SUCCESS) throw std::runtime_error(libusb_error_name(status));

    if (length != (uint32_t)transferred) {
        std::stringstream sstr;
        sstr << "Did transfer " << transferred << " but " << length << " was requested!";
        throw std::runtime_error(sstr.str());
    }

    // Read result; in inline mode also ackDisabled packets will send a response.
    status = libusb_bulk_transfer(
        m_handle,
        /* endpoint*/ (0x81 | LIBUSB_ENDPOINT_IN),
        (unsigned char*)&result,
        sizeof(result),
        &transferred,
        /*timeout*/ 10);
    
    if (status == LIBUSB_ERROR_TIMEOUT) 
        std::cerr << "USB readback timeout" << std::endl;
    
    if (status != LIBUSB_SUCCESS) 
        std::cerr << "USB readback failed: " << libusb_error_name(status) << std::endl;
}

void Crazyradio::setPower(Power power)
{
    sendVendorSetup(SET_RADIO_POWER, power, 0, NULL, 0);
}

void Crazyradio::setArc(uint8_t arc)
{
    sendVendorSetup(SET_RADIO_ARC, arc, 0, NULL, 0);
}

void Crazyradio::setArdTime(uint16_t us)
{
    // Auto Retransmit Delay:
    // 0000 - Wait 250uS
    // 0001 - Wait 500uS
    // 0010 - Wait 750uS
    // ........
    // 1111 - Wait 4000uS

    // Round down, to value representing a multiple of 250uS
    int t = (us / 250) - 1;
    if (t < 0) {
        t = 0;
    }
    if (t > 0xF) {
        t = 0xF;
    }
    sendVendorSetup(SET_RADIO_ARD, t, 0, NULL, 0);
}

void Crazyradio::setArdBytes(uint8_t nbytes)
{
    sendVendorSetup(SET_RADIO_ARD, 0x80 | nbytes, 0, NULL, 0);
}

void Crazyradio::setContCarrier(bool active)
{
    sendVendorSetup(SET_CONT_CARRIER, active, 0, NULL, 0);
}

void Crazyradio::setInlineMode(bool enable)
{
    sendVendorSetup(SET_INLINE_MODE, enable, 0, NULL, 0);
}

void Crazyradio::ackToCrtpPacket(Ack * ack, libcrtp::CrtpPacket * packet)
{
    packet->port = (libcrtp::CrtpPort)((ack->data[0] >> 4) & 0xF);
    packet->channel = ack->data[0] & 0b11;
    for (int i = 0; i < ack->total_length - 3; i++) // total_length includes the header (2bytes) and channel/port byte
        packet->data[i] = ack->data[i + 1];
    packet->dataLength = ack->total_length - 3;
}




} // namepsace libcrazyradio
