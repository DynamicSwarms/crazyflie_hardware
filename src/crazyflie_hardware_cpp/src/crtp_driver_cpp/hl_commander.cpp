#include "crazyflie_hardware_cpp/crtp_driver_cpp/hl_commander.hpp"
using std::placeholders::_1;

HighLevelCommanderDriver::HighLevelCommanderDriver(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link)
    : HighLevelCommanderLogic(link)
    , node(node)
{
    land_sub = node->create_subscription<crazyflie_interfaces::msg::Land>(
                "~/land", 10,
                std::bind(&HighLevelCommanderDriver::land_callback, this, _1));

    takeoff_sub = node->create_subscription<crazyflie_interfaces::msg::Takeoff>(
                "~/takeoff", 10,
                std::bind(&HighLevelCommanderDriver::takeoff_callback, this, _1));

};

void HighLevelCommanderDriver::land_callback(const crazyflie_interfaces::msg::Land::SharedPtr msg)
{
    RCLCPP_WARN(node->get_logger(), "%f", msg->height);
    HighLevelCommanderLogic::send_takeoff(msg->height, 1.0, msg->group_mask, msg->yaw);
}

void HighLevelCommanderDriver::takeoff_callback(const crazyflie_interfaces::msg::Takeoff::SharedPtr msg)
{
    HighLevelCommanderLogic::send_land(msg->height, 1.0, msg->group_mask, msg->yaw);
}