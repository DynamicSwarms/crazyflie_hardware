#include "libcrazyradio/Crazyradio.hpp"


#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cstring>

#include <libusb-1.0/libusb.h>
namespace libcrazyradio {
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
    LAUNCH_BOOTLOADER   = 0xFF,
};

Crazyradio::Crazyradio(uint32_t device_id) 
    : USBDevice(0x1915, 0x7777),
    m_datarate(Datarate_250KPS),
    m_address(0xDEADBEEF),
    m_channel(0),
    m_ackEnable(false) // need to be differnt in order to initially set
{
    open(device_id);
    setDatarate(Datarate_2MPS);
    setChannel(2);
    setContCarrier(false);
    setAddress(0xE7E7E7E7E7);
    setPower(Power_0DBM);
    setArc(3);
    setArdBytes(32);
    setAckEnable(true);
}

Crazyradio::~Crazyradio()
{
}



void Crazyradio::sendCrtpPacket(
        libcrtp::CrtpLink * link,
        libcrtp::CrtpPacket * packet,
        Ack & result)
{
    uint8_t data[32];
    data[0] = packet->port << 4 | packet->channel;
    memcpy(&data[1], &packet->data, packet->dataLength);

    setToCrtpLink(link);
    sendPacket(data, 1 + packet->dataLength , result);
}

void Crazyradio::setToCrtpLink(libcrtp::CrtpLink * link)
{
    setChannel(link->getChannel());
    setAddress(link->getAddress());
    switch (link->getDatarate()) 
    {
        case 2: 
            setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_2MPS);
            break;
        case 1: 
            setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_1MPS);
            break;
        default: 
            setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_250KPS);
    }  
}

void Crazyradio::setChannel(uint8_t channel)
{
    if (m_channel != channel)
    {
        sendVendorSetup(SET_RADIO_CHANNEL, channel, 0, NULL, 0);
        m_channel = channel;
    }
}

void Crazyradio::setAddress(uint64_t address)
{
    if (m_address != address) {
        unsigned char a[5];
        a[4] = (address >> 0) & 0xFF;
        a[3] = (address >> 8) & 0xFF;
        a[2] = (address >> 16) & 0xFF;
        a[1] = (address >> 24) & 0xFF;
        a[0] = (address >> 32) & 0xFF;

        //sendVendorSetup(SET_RADIO_ADDRESS, 0, 0, a, 5);
        // unsigned char a[] = {0xe7, 0xe7, 0xe7, 0xe7, 0x02};

        /*int status = */libusb_control_transfer(
            m_handle,
            LIBUSB_REQUEST_TYPE_VENDOR,
            SET_RADIO_ADDRESS,
            0,
            0,
            a,
            5,
            /*timeout*/ 1000);
         //if (status != LIBUSB_SUCCESS) {
         //    std::cerr << "sendVendorSetup: " << libusb_error_name(status) << std::endl;
         //}
        m_address = address;
    }
}

void Crazyradio::setDatarate(Datarate datarate)
{
    if (m_datarate != datarate)
    {
        sendVendorSetup(SET_DATA_RATE, datarate, 0, NULL, 0);
        m_datarate = datarate;
    }
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

void Crazyradio::setAckEnable(bool enable)
{
    if (m_ackEnable != enable) 
    {
        sendVendorSetup(ACK_ENABLE, enable, 0, NULL, 0);
        m_ackEnable = enable;
    }
}

void Crazyradio::setContCarrier(bool active)
{
    sendVendorSetup(SET_CONT_CARRIER, active, 0, NULL, 0);
}


void Crazyradio::sendPacket(
    const uint8_t * data,
    uint32_t length,
    Ack& result
)
{
    result.ack = false;
    result.size = 0;

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

    // Read result
    status = libusb_bulk_transfer(
        m_handle,
        /* endpoint*/ (0x81 | LIBUSB_ENDPOINT_IN),
        (unsigned char*)&result,
        sizeof(result) - 1,
        &transferred,
        /*timeout*/ 100);
    
    if (status == LIBUSB_ERROR_TIMEOUT) std::cerr << "timeout in read\n";
    
    if (status != LIBUSB_SUCCESS) 
    {
        // TODO: On Node restart throwing this error will block 
       // throw std::runtime_error(libusb_error_name(status));
    } 

    result.size = transferred - 1;

    //if (m_enableLogging) {
    //    logAck(result);
    //}
}


void Crazyradio::ackToCrtpPacket(Ack * ack, libcrtp::CrtpPacket * packet)
{
    packet->port = (libcrtp::CrtpPort)((ack->data[0] >> 4) & 0xF);
    packet->channel = ack->data[0] & 0b11;
    for (int i = 0; i < ack->size; i++) 
        packet->data[i] = ack->data[i+1];
    packet->dataLength = ack->size -1;
}




} // namepsace libcrazyradio