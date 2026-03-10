#include "crazyflie_hardware/crtp_driver_cpp/platform.hpp"

Platform::Platform(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    CrtpLink *link)
    : PlatformLogic(link)
    , m_logger(node_logging_interface->get_logger().get_child("Platform"))
{
    (void)node_base_interface;
    int protocol;
    std::string firmware;
    std::string device_type;

    if (get_protocol(protocol) && get_firmware(firmware) && get_device_type(device_type)) {
        RCLCPP_INFO(m_logger, "Connected to Crazyflie with protocol %d, firmware %s and device type %s", protocol, firmware.c_str(), device_type.c_str());
    }

    RCLCPP_DEBUG(m_logger, "Platform initialized");
}