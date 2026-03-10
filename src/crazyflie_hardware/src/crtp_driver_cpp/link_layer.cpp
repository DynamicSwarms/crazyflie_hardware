#include "crazyflie_hardware/crtp_driver_cpp/link_layer.hpp"

LinkLayer::LinkLayer(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    CrtpLink *link)
    : LinkLayerLogic(link)
    , m_logger(node_logging_interface->get_logger().get_child("LinkLayer"))
{
    (void)node_base_interface;
    RCLCPP_DEBUG(m_logger, "LinkLayer initialized");
}