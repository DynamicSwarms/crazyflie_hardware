#include "crazyflie_hardware/crtp_driver_cpp/console.hpp"

Console::Console(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<CrtpLink>link)
    : ConsoleLogic(link)
    , m_logger(node_logging_interface->get_logger().get_child("Console"))
{
    callback_group = node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    console_publisher = rclcpp::create_publisher<std_msgs::msg::String>(
        node_topics_interface,
        "~/console",
        10);

    set_message_callback([logger = m_logger, publisher = console_publisher](const std::string &message) {
        RCLCPP_WARN(logger, "%s", message.c_str());
        auto msg = std_msgs::msg::String();
        msg.data = message;
        publisher->publish(msg);
    });

    for (int i = 0; i < 5; i++) ConsoleLogic::send_consolepacket(); // This initializes communication with the Crazyflie.
    RCLCPP_DEBUG(m_logger, "Console initialized");
}
