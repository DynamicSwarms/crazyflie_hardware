#include "crazyflie_hardware_cpp/crtp_driver_cpp/console.hpp"
using std::placeholders::_1;

Console::Console(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink *link)
    : ConsoleLogic(link), node(node)
{
    RCLCPP_INFO(node->get_logger(), "Console");

    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    console_publisher = node->create_publisher<std_msgs::msg::String>(
        "~/console",
        10);

    for (int i = 0; i < 10; i++)
    {
        ConsoleLogic::send_consolepacket();
    }

    RCLCPP_INFO(node->get_logger(), "Console initialized");
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
            RCLCPP_WARN(node->get_logger(), "%s", message.c_str());
            this->console_message(message);
            message.clear();
        }
    }
}

void Console::console_message(const std::string)
{
    return;
}