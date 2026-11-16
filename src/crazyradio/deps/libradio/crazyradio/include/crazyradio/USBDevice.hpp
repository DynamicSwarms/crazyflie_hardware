#pragma once

#include <stdint.h>
#include <utility>
#include <memory>
#include <vector>

// forward declarations
struct libusb_context;
struct libusb_device;
struct libusb_device_handle;

class USBDevice
{
public:
  USBDevice(
    uint16_t idVendor,
    uint16_t idProduct);

  virtual ~USBDevice();

protected:
    static uint32_t numDevices(
      uint16_t idVendor,
      uint16_t idProduct);

    std::vector<std::shared_ptr<libusb_device>> enumerate();
    bool tryOpenAndClaim(libusb_device *device);

    void sendVendorSetup(
        uint8_t request,
        uint16_t value,
        uint16_t index,
        const unsigned char* data,
        uint16_t length);

public:
  const std::pair<int, int> version() const
  {
    return std::make_pair(m_versionMajor, m_versionMinor);
  }

protected:
    libusb_context* m_ctx;
    libusb_device_handle *m_handle;

    int m_versionMajor;
    int m_versionMinor;

private:
  uint16_t m_idVendor;
  uint16_t m_idProduct;
};