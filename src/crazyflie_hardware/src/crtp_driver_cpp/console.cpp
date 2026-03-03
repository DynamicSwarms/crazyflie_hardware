#include "crazyflie_hardware/crtp_driver_cpp/console.hpp"
using std::placeholders::_1;

Console::Console(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    CrtpLink *link)
    : ConsoleLogic(link)
    , m_logger(node_logging_interface->get_logger().get_child("Console"))
{
    callback_group = node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    console_publisher = rclcpp::create_publisher<std_msgs::msg::String>(
        node_topics_interface,
        "~/console",
        10);

    for (int i = 0; i < 5; i++) ConsoleLogic::send_consolepacket(); // This initializes communication with the Crazyflie.
    RCLCPP_DEBUG(m_logger, "Console initialized");
}

void Console::crtp_response_callback(const CrtpPacket &packet)
{
    static std::string message("");
    // A message can be from multiple messages.
    // A \n ends one message.
    if (packet.channel == CHANNEL_CONSOLE)
    {
        std::string string((char *)packet.data, packet.data_length);

        message.append(string);

        if (message.back() == '\n' || message.back() == '\r')
        {
            message.pop_back(); // Do not print newline twice.
            RCLCPP_WARN(m_logger, "%s", message.c_str());
            this->console_message(message);
            message.clear();
        }
    }
}

void Console::console_message(const std::string message)
{
    auto msg = std_msgs::msg::String();
    msg.data = message;
    console_publisher->publish(msg);
}