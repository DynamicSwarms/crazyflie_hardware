#pragma once

#include <stdint.h>
#include <map>
#include <tuple>
#include <array>
#include <optional>
#include <mutex>
#include <functional>
#include <utility>
#include "InlineAck.hpp"
#include "radio_logger.hpp"
#include "USBDevice.hpp"
#include "interface/IRadio.hpp"

#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"

namespace libradio::crazyradio {
class Crazyradio : public libradio::IRadio, public USBDevice
{
public:
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
    Crazyradio(bool logToFile, bool logToSocket, uint8_t channel,
        const std::filesystem::path & logRoot);
    
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
    // Average valid RSSI in the current quality window; unavailable without samples.
    std::optional<double> getLinkRssi(const libcrtp::CrtpLinkIdentifier * link) const override;
        
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
        void addSample(double delivery, std::optional<int> rssi);

        std::array<double, 64> samples{};
        uint8_t next = 0;
        uint8_t count = 0;
        double sum = 0.0;
        std::array<std::optional<int>, 64> rssiSamples{};
        int rssiSum = 0;
        uint8_t rssiCount = 0;
    };

    SafeLinkKey safeLinkKey(const libcrtp::CrtpLinkIdentifier * link) const;
    bool enableSafeLink(const libcrtp::CrtpLinkIdentifier * link);

    void recordCommunication(const libcrtp::CrtpLinkIdentifier * link,
        const libcrtp::CrtpPacket * packet, const InlineAck & ack);

    void sendPacketInline(
        const uint8_t* data,
        uint32_t length, 
        Datarate datarate,
        uint8_t channel,
        uint64_t address,
        bool ackEnabled,
        InlineAck& result
    );

    void sendPacket(
        const uint8_t* data,
        uint32_t length, 
        InlineAck& result
    );

    void setPower(Power power);

    void setArc(uint8_t arc);

    void setArdTime(uint16_t us);

    void setArdBytes(uint8_t nbytes);

    void setContCarrier(bool active);

    void setInlineMode(uint8_t mode);

    void ackToCrtpPacket(const InlineAck * ack, libcrtp::CrtpPacket * packet);

    RadioLogger m_radioLogger;
    uint64_t m_transferIndex{0};
    mutable std::mutex m_stateMutex;
    std::map<SafeLinkKey, SafeLinkState> m_safeLinkStates;
    std::map<SafeLinkKey, LinkQuality> m_linkQualities;
};



} // namespace libcrazyradio
