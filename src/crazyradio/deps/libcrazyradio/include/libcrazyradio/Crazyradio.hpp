#pragma once

#include <stdint.h>
#include "USBDevice.hpp"


#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"

namespace libcrazyradio {
class Crazyradio : public USBDevice
{
public:
    struct Ack
    {
        Ack()
        : ack(0)
        {}

        uint8_t total_length; // total length including the header
        uint8_t ack:1;
        uint8_t rssi:1;
        uint8_t invalid_settings:1;
        uint8_t reserved:1;
        uint8_t retry:4;
        uint8_t data[31];
        uint8_t buffer;
    }__attribute__((packed));

    enum Datarate
    {
        Datarate_250KPS = 0,
        Datarate_1MPS   = 1,
        Datarate_2MPS   = 2,
    };

    enum Power
    {
        Power_M18DBM = 0,
        Power_M12DBM = 1,
        Power_M6DBM  = 2,
        Power_0DBM   = 3,
    };

public:
    Crazyradio();
    
    virtual ~Crazyradio();

    /**
     * Transmits a CrtpPacket over the Crazyradio.
     * Returns true if the packet was sent successfully.
     * If the link is non broadcast and the packet was sent successfully, the responsePacket will contain a response from the Crazyflie.
    */
    bool sendCrtpPacket(
        libcrtp::CrtpLinkIdentifier * link,
        libcrtp::CrtpPacket * packet,
        libcrtp::CrtpPacket * responsePacket);
        
private:
    void sendPacketInline(
        const uint8_t* data,
        uint32_t length, 
        Datarate datarate,
        uint8_t channel,
        uint64_t address,
        bool ackEnabled,
        Ack& result
    );

    void sendPacket(
        const uint8_t* data,
        uint32_t length, 
        bool ackEnabled,
        Ack& result
    );

    void setPower(Power power);

    void setArc(uint8_t arc);

    void setArdTime(uint16_t us);

    void setArdBytes(uint8_t nbytes);

    void setContCarrier(bool active);

    void setInlineMode(bool enable);

    void ackToCrtpPacket(Ack * ack, libcrtp::CrtpPacket * packet);
};



} // namespace libcrazyradio