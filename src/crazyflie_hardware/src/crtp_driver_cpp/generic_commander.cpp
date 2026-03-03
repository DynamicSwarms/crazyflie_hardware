#include "crazyflie_hardware/crtp_driver_cpp/generic_commander.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

GenericCommander::GenericCommander(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    CrtpLink *link)
    : GenericCommanderLogic(link)
    , m_logger(node_logging_interface->get_logger().get_child("GenericCommander"))
    , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;

    m_cmd_position_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::Position>(
        node_topics_interface,
        "~/cmd_position",
        10,
        std::bind(&GenericCommander::cmd_position_callback, this, _1),
        sub_opt);

    m_notify_setpoints_stop_service = rclcpp::create_service<crazyflie_interfaces::srv::NotifySetpointsStop>(
        node_base_interface,
        node_services_interface,
        "~/notify_setpoints_stop",
        std::bind(&GenericCommander::notify_setpoints_stop_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );
        
    RCLCPP_DEBUG(m_logger, "Generic Commander initialized");
};

void GenericCommander::cmd_position_callback(const crazyflie_interfaces::msg::Position::SharedPtr msg)
{
    GenericCommanderLogic::send_position_setpoint(msg->x, msg->y, msg->z, msg->yaw);
}

void 
GenericCommander::notify_setpoints_stop_service(
    const crazyflie_interfaces::srv::NotifySetpointsStop::Request::SharedPtr request,
    crazyflie_interfaces::srv::NotifySetpointsStop::Response::SharedPtr response)
{    
    GenericCommanderLogic::send_notify_setpoints_stop(request->remain_valid_millisecs);
}