#pragma once

#include <stdint.h>
#include "USBDevice.hpp"


#include "libcrazyradio/CrtpPacket.hpp"
#include "libcrazyradio/CrtpLink.hpp"

namespace libcrazyradio {
class Crazyradio : public USBDevice
{
public:
    struct Ack
    {
        Ack()
        : ack(0)
        , size(0)
        {}

        uint8_t ack:1;
        uint8_t powerDet:1;
        uint8_t retry:4;
        uint8_t data[32];

        uint8_t size;
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
    Crazyradio(uint32_t device_id);

    virtual ~Crazyradio();

    void sendCrtpPacket(
        libcrtp::CrtpLinkIdentifier * link,
        libcrtp::CrtpPacket * packet,
        Ack & result);
        
    void sendPacket(
        const uint8_t* data,
        uint32_t length, 
        Ack& result
    );

    static void ackToCrtpPacket(Ack * ack, libcrtp::CrtpPacket * packet);
private:
    void setToCrtpLink(libcrtp::CrtpLinkIdentifier * link);

    void setChannel(uint8_t channel);

    void setAddress(uint64_t address);

    void setDatarate(Datarate datarate);

    void setPower(Power power);

    void setArc(uint8_t arc);

    void setArdTime(uint16_t us);

    void setArdBytes(uint8_t nbytes);

    void setAckEnable(bool enable);

    void setContCarrier(bool active);
       
private: 
    uint8_t m_channel;
    uint64_t m_address;
    Datarate m_datarate;
    bool m_ackEnable;

};



} // namespace libcrazyradio