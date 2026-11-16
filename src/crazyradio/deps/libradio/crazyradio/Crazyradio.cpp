#include "crazyradio/Crazyradio.hpp"


#include <sstream>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <cstring>

#include <libusb-1.0/libusb.h>
namespace libradio::crazyradio {
namespace {
Crazyradio::Datarate toCrazyradioDatarate(uint8_t datarate)
{
    switch (datarate) {
        case 0: return Crazyradio::Datarate_250KPS;
        case 1: return Crazyradio::Datarate_1MPS;
        case 2: return Crazyradio::Datarate_2MPS;
        default:
            throw std::invalid_argument("Invalid link data rate: " + std::to_string(datarate));
    }
}
} // namespace

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

Crazyradio::Crazyradio(bool logToFile, bool logToSocket, uint8_t channel,
    const std::filesystem::path & logRoot)
    : IRadio(logToFile, logToSocket, channel, logRoot, "USB")
    , USBDevice(0x1915, 0x7777)
    , m_radioLogger(logToFile, m_logger.directory(), channel)
{
    bool success = false;
    auto devices = enumerate();
    for (size_t deviceId = 0; deviceId < devices.size(); deviceId++) {
        if (tryOpenAndClaim(devices[deviceId].get())) {
            success = true;
            break;
        }
    }
    if (!success) {
        throw std::runtime_error("No available Crazyradio device found.");
    }
    
    setContCarrier(false);
    setPower(Power_0DBM);
    setArc(1); // libcrtp handles retries, however most transmission errors are fixed by the first retry -> avoid to many backoffs
    setArdBytes(32);
    
    const auto version = this->version();
    const bool is_supported_crazyradio2 =
        (version.first > 5 || (version.first == 5 && version.second >= 3)) && // Crazyradio 2.0 with inline mode 2 firmware
        version.first != 0x99; // but not a Crazyradio PA
    if (!is_supported_crazyradio2) {
        throw std::runtime_error("Unsupported Crazyradio version: " + std::to_string(version.first) + "." + std::to_string(version.second) 
        + "Inline mode with RSSI requires Crazyradio 2.0 firmware >= 5.3.");
    }

    setInlineMode(2);

    std::cerr << "Crazyradio USB initialized with RSSI Inline Mode 2 enabled." << std::endl;
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
    std::lock_guard<std::mutex> lock(m_stateMutex);

    if (packet->dataLength > sizeof(packet->data))
        throw std::runtime_error("CRTP payload exceeds 31 bytes");

    SafeLinkState * safeLink = nullptr;
    if (!link->isBroadcast) {
        auto& state = m_safeLinkStates[safeLinkKey(link)];
        if (!state.initialized) {
            state.enabled = enableSafeLink(link);
            state.initialized = true;
            state.up = 0;
            state.down = 0;
        }
        safeLink = &state;
    }

    uint8_t data[32];
    data[0] = packet->port << 4 | (packet->channel & 0x03);
    if (safeLink && safeLink->enabled) data[0] |= (safeLink->up << 3) | (safeLink->down << 2);
    memcpy(&data[1], &packet->data, packet->dataLength);

    InlineAck ack;
    sendPacketInline(
        data,
        1 + packet->dataLength, 
        toCrazyradioDatarate(link->datarate),
        link->channel,
        link->address,
        ! link->isBroadcast,
        ack
    );
    recordCommunication(link, packet, ack);

    if (ack.usb_reply_malformed || ack.invalid_settings) return false;
    if (link->isBroadcast) return true;
    if (!ack.ack) return false;

    if (safeLink && safeLink->enabled) 
        safeLink->up ^= 1;

    if (ack.data_length == 0)
    {
        memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
        std::cerr << "Empty ACK data. Fabricated null packet." << std::endl;
        return true;
    }

    if (safeLink && safeLink->enabled) 
    {
        if (((ack.data[0] >> 2) & 0x01) == safeLink->down)
        {
            safeLink->down ^= 1;
            ack.data[0] &= 0xF3;
        } else {
            // If the down counter does not match, this is a reapeted ack and the payload needs to be dropped
            memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
            std::cerr << "SafeLink: Down bit mismatch. Fabricated null packet." << std::endl;
            return true;
        }
    }

    ackToCrtpPacket(&ack, responsePacket);
    return true;
}

void Crazyradio::recordCommunication(
    const libcrtp::CrtpLinkIdentifier * link,
    const libcrtp::CrtpPacket * packet,
    const InlineAck & ack)
{
    const Transfer transfer{++m_transferIndex, std::chrono::system_clock::now()};
    libcrtp::CrtpPacket receivedPacket{};
    const bool acknowledged = ack.ack && !ack.usb_reply_malformed && !ack.invalid_settings;
    const bool responseValid = !link->isBroadcast && acknowledged && ack.data_length > 0;

    if (responseValid) ackToCrtpPacket(&ack, &receivedPacket);
    m_logger.log(link, packet, &receivedPacket, responseValid, transfer);
    m_radioLogger.log(transfer, ack, !link->isBroadcast);

    if (link->isBroadcast) return;

    const double delivery = acknowledged ? 1.0 / (ack.retry + 1.0) : 0.0;
    const std::optional<int> rssi = acknowledged && ack.rssi_valid ? std::optional<int>(ack.rssi_dbm) : std::nullopt;

    m_linkQualities[safeLinkKey(link)].addSample(delivery, rssi);
}

void Crazyradio::LinkQuality::addSample(
    double delivery,
    std::optional<int> rssi)
{
    // Replace the delivery sample. Unused slots contain zero.
    sum -= samples[next];
    samples[next] = delivery;
    sum += delivery;
    if (count < samples.size()) ++count;

    // Replace the RSSI sample, if present.
    if (rssiSamples[next]) {
        rssiSum -= *rssiSamples[next];
        --rssiCount;
    }

    rssiSamples[next] = rssi;
    if (rssi) {
        rssiSum += *rssi;
        ++rssiCount;
    }

    next = (next + 1) % samples.size();
}


Crazyradio::SafeLinkKey Crazyradio::safeLinkKey(
    const libcrtp::CrtpLinkIdentifier * link) const
{
    return {link->channel, link->address, link->datarate};
}

bool Crazyradio::enableSafeLink(
    const libcrtp::CrtpLinkIdentifier * link)
{
    const libcrtp::CrtpPacket requestPacket{libcrtp::LINK_LAYER, 0x0F, {0x05, 0x01}, 2, false, 0, false, {}};
    // Deliberately "wrong" channel, however logs show it
    // Do not mask channel with 0x03 here: negotiation must send FF 05 01.
    const uint8_t request[] = {
        static_cast<uint8_t>(requestPacket.port << 4 | requestPacket.channel),
        requestPacket.data[0], 
        requestPacket.data[1]};

    for (int attempt = 0; attempt < 10; ++attempt) {
        InlineAck ack;
        sendPacketInline(
            request,
            sizeof(request),
            toCrazyradioDatarate(link->datarate),
            link->channel,
            link->address,
            true,
            ack);
        recordCommunication(link, &requestPacket, ack);

        if (ack.ack && !ack.usb_reply_malformed && !ack.invalid_settings && ack.data_length == 3 &&
            ack.data[0] == 0xFF && ack.data[1] == 0x05 && ack.data[2] == 0x01) {
            return true;
        }
    }

    std::cerr << "SafeLink negotiation failed after 10 attempts for cf" 
              << std::to_string(link->address & 0xFF) + " on channel " + std::to_string(link->channel)
              << std::endl;
    return false;
}

void Crazyradio::resetLink(const libcrtp::CrtpLinkIdentifier * link)
{
    std::lock_guard<std::mutex> lock(m_stateMutex);
    std::cerr << "Link Reset for CF 0x" << std::hex << (int)(uint8_t)(link->address & 0xFF) << std::dec << std::endl;
    m_safeLinkStates.erase(safeLinkKey(link));
    m_linkQualities.erase(safeLinkKey(link));
}

double Crazyradio::getLinkQuality(const libcrtp::CrtpLinkIdentifier * link) const
{
    std::lock_guard<std::mutex> lock(m_stateMutex);
    if (link->isBroadcast) return 1.0;
    const auto quality = m_linkQualities.find(safeLinkKey(link));
    if (quality == m_linkQualities.end() || quality->second.count == 0) return 1.0;
    return quality->second.sum / quality->second.count;
}

std::optional<double> Crazyradio::getLinkRssi(const libcrtp::CrtpLinkIdentifier * link) const
{
    std::lock_guard<std::mutex> lock(m_stateMutex);
    if (link->isBroadcast) return std::nullopt;
    const auto quality = m_linkQualities.find(safeLinkKey(link));
    if (quality == m_linkQualities.end() || quality->second.rssiCount == 0) return std::nullopt;
    return static_cast<double>(quality->second.rssiSum) / quality->second.rssiCount;
}

void Crazyradio::sendPacketInline(
    const uint8_t* data,
    uint32_t length, 
    Datarate datarate,
    uint8_t channel,
    uint64_t address,
    bool ackEnabled,
    InlineAck& result
)
{
    if (length > 32) throw std::runtime_error("Radio payload exceeds 32 bytes");

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

    sendPacket(inlineData, 8 + length, result);
}

void Crazyradio::sendPacket(
    const uint8_t * data,
    uint32_t length,
    InlineAck& result
)
{
    result = InlineAck{};

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
    transferred = 0;
    uint8_t reply[64]{};
    status = libusb_bulk_transfer(
        m_handle,
        /* endpoint*/ (0x81 | LIBUSB_ENDPOINT_IN),
        reply,
        sizeof(reply),
        &transferred,
        /*timeout*/ 10);
        
    if (status != LIBUSB_SUCCESS) 
        throw std::runtime_error("USB readback failed: " + std::string(libusb_error_name(status)));
    result = decodeInlineAck(reply, static_cast<std::size_t>(transferred));
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

void Crazyradio::setInlineMode(uint8_t mode)
{
    sendVendorSetup(SET_INLINE_MODE, mode, 0, NULL, 0);
}

void Crazyradio::ackToCrtpPacket(const InlineAck * ack, libcrtp::CrtpPacket * packet)
{
    if (ack->usb_reply_malformed || ack->data_length == 0 || ack->data_length > 32)
        throw std::runtime_error("Invalid ACK payload length");

    packet->port = (libcrtp::CrtpPort)((ack->data[0] >> 4) & 0xF);
    packet->channel = ack->data[0] & 0b11;
    for (int i = 0; i < ack->data_length - 1; i++)
        packet->data[i] = ack->data[i + 1];
    packet->dataLength = ack->data_length - 1;
}




} // namepsace libcrazyradio
