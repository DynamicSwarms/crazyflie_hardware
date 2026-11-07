#include "crazyradio/USBDevice.hpp"

#include <sstream>
#include <stdexcept>

#include <libusb-1.0/libusb.h>

USBDevice::USBDevice(
    uint16_t idVendor,
    uint16_t idProduct)
    : m_ctx(NULL)
    , m_handle(NULL)
    , m_idVendor(idVendor)
    , m_idProduct(idProduct)
{
    int result = libusb_init(&m_ctx);
    if (result != LIBUSB_SUCCESS) {
        throw std::runtime_error(libusb_error_name(result));
    }
}

USBDevice::~USBDevice()
{
    if (m_handle) {
        // ignore the error (we don't want to throw in dtor)
        libusb_release_interface(m_handle, 0);

        // function returns void => no error checking
        libusb_close(m_handle);
    }

    // function returns void => no error checking
    libusb_exit(m_ctx);
}

uint32_t USBDevice::numDevices(
  uint16_t idVendor,
  uint16_t idProduct)
{
    libusb_context* ctx;
    int result = libusb_init(&ctx);
    if (result != LIBUSB_SUCCESS) {
        throw std::runtime_error(libusb_error_name(result));
    }

    // discover devices
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(NULL, &list);
    ssize_t i = 0;
    uint32_t num = 0;
    int err = 0;
    if (cnt < 0) {
        throw std::runtime_error("Error during get_device_list");
    }
    for (i = 0; i < cnt; i++) {
        libusb_device *device = list[i];
        libusb_device_descriptor deviceDescriptor;
        err = libusb_get_device_descriptor(device, &deviceDescriptor);
        if (err != LIBUSB_SUCCESS) {
            libusb_free_device_list(list, 1);
            throw std::runtime_error(libusb_error_name(err));
        }
        else if (deviceDescriptor.idVendor == idVendor &&
                 deviceDescriptor.idProduct == idProduct) {
            ++num;
        }
    }
    libusb_free_device_list(list, 1);

    // function returns void => no error checking
    libusb_exit(ctx);

    return num;
}

std::vector<std::shared_ptr<libusb_device>> USBDevice::enumerate()
{
    // discover devices
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(m_ctx, &list);
    ssize_t i = 0;
    int err = 0;
    if (cnt < 0) {
        throw std::runtime_error("Error during get_device_list");
    }

    std::vector<std::shared_ptr<libusb_device>> devices;
    try {
        for (i = 0; i < cnt; i++) {
            libusb_device *device = list[i];
            libusb_device_descriptor deviceDescriptor;
            err = libusb_get_device_descriptor(device, &deviceDescriptor);
            if (err != LIBUSB_SUCCESS) {
                throw std::runtime_error(libusb_error_name(err));
            }
            else if (deviceDescriptor.idVendor == m_idVendor &&
                     deviceDescriptor.idProduct == m_idProduct) {
                devices.push_back(std::shared_ptr<libusb_device>(
                    libusb_ref_device(device), libusb_unref_device));
            }
        }
    } catch (...) {
        libusb_free_device_list(list, 1);
        throw;
    }
    libusb_free_device_list(list, 1);
    return devices;
}

bool USBDevice::tryOpenAndClaim(libusb_device *device)
{
    libusb_device_descriptor deviceDescriptor;
    int err = libusb_get_device_descriptor(device, &deviceDescriptor);
    if (err != LIBUSB_SUCCESS) {
        throw std::runtime_error(libusb_error_name(err));
    }

    libusb_device_handle *handle = NULL;
    err = libusb_open(device, &handle);
    if (err == LIBUSB_ERROR_BUSY) return false;
    if (err != LIBUSB_SUCCESS) {
        throw std::runtime_error(libusb_error_name(err));
    }

    // configure
    err = libusb_set_configuration(handle, 1);
    if (err == LIBUSB_SUCCESS) {
        err = libusb_claim_interface(handle, 0);
    }
    if (err != LIBUSB_SUCCESS) {
        libusb_close(handle);
        if (err == LIBUSB_ERROR_BUSY) return false;
        throw std::runtime_error(libusb_error_name(err));
    }

    m_handle = handle;
    m_versionMajor = deviceDescriptor.bcdDevice >> 8;
    m_versionMinor = deviceDescriptor.bcdDevice & 0xFF;
    return true;
}

void USBDevice::sendVendorSetup(
    uint8_t request,
    uint16_t value,
    uint16_t index,
    const unsigned char* data,
    uint16_t length)
{
    if (!m_handle) {
        throw std::runtime_error("No valid device handle!");
    }
    int status = libusb_control_transfer(
        m_handle,
        LIBUSB_REQUEST_TYPE_VENDOR,
        request,
        value,
        index,
        (unsigned char*)data,
        length,
        /*timeout*/ 1000);    
    
    if (status != LIBUSB_SUCCESS) {
        throw std::runtime_error(libusb_error_name(status));
    }
}