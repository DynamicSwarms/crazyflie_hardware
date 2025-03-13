#include "crazyflie_hardware_cpp/crtp_driver_cpp/console.hpp"
using std::placeholders::_1;

Console::Console(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link)
    : ConsoleLogic(link)
    , node(node)
{
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    console_publisher = node->create_publisher<std_msgs::msg::String>(
        "~/console",
        10);

    for (int i = 0; i < 10; i++) {
        ConsoleLogic::send_consolepacket();
    }

    RCLCPP_WARN(node->get_logger(), "Console initialized");
}

void Console::console_message(const std::string)
{
    return;
}