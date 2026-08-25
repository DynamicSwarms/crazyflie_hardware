#pragma once

#include <stdint.h>
#include <map>
#include <tuple>
#include <array>
#include "USBDevice.hpp"
#include "interface/IRadio.hpp"

#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"

namespace libradio::crazyradio {
class Crazyradio : public libradio::IRadio, public USBDevice
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
        uint8_t data[32];
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
        const libcrtp::CrtpLinkIdentifier * link,
        const libcrtp::CrtpPacket * packet,
        libcrtp::CrtpPacket * responsePacket) override;

    void resetLink(const libcrtp::CrtpLinkIdentifier * link) override;

    double getLinkQuality(const libcrtp::CrtpLinkIdentifier * link) const override;
        
private:
    struct SafeLinkState
    {
        bool initialized = false;
        bool enabled = false;
        uint8_t up = 0;
        uint8_t down = 0;
    };

    using SafeLinkKey = std::tuple<uint8_t, uint64_t, uint8_t>;

    struct LinkQuality
    {
        std::array<double, 64> samples{};
        uint8_t next = 0;
        uint8_t count = 0;
        double sum = 0.0;
    };

    SafeLinkKey safeLinkKey(const libcrtp::CrtpLinkIdentifier * link) const;
    bool enableSafeLink(const libcrtp::CrtpLinkIdentifier * link, Datarate datarate);

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

    std::map<SafeLinkKey, SafeLinkState> m_safeLinkStates;
    std::map<SafeLinkKey, LinkQuality> m_linkQualities;
};



} // namespace libcrazyradio
