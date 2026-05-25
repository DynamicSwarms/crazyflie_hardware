#include "crazyflie_hardware/crtp_driver_cpp/generic_commander.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

GenericCommander::GenericCommander(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    CrtpLink *link)
    : GenericCommanderLogic(link)
    , m_logger(node_logging_interface->get_logger().get_child("GenericCommander"))
    , m_node_parameters_interface(node_parameters_interface)
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

    m_cmd_velocity_world_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::VelocityWorld>(
        node_topics_interface,
        "~/cmd_velocity_world",
        10,
        std::bind(&GenericCommander::cmd_velocity_world_callback, this, _1),
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

void 
GenericCommander::cmd_position_callback(const crazyflie_interfaces::msg::Position::SharedPtr msg)
{
    GenericCommanderLogic::send_position_setpoint(msg->x, msg->y, msg->z, msg->yaw);
}

void 
GenericCommander::cmd_velocity_world_callback(const std::shared_ptr<crazyflie_interfaces::msg::VelocityWorld> msg)
{
    if (m_get_controller_parameter() == 2) RCLCPP_WARN(m_logger, "Velocity control in world frame is not compatible with MEL controller. Please switch to a compatible controller in the parameters.");
    GenericCommanderLogic::send_velocity_world_setpoint(msg->vel.x, msg->vel.y, msg->vel.z, msg->yaw_rate);
}


void 
GenericCommander::notify_setpoints_stop_service(
    const crazyflie_interfaces::srv::NotifySetpointsStop::Request::SharedPtr request,
    crazyflie_interfaces::srv::NotifySetpointsStop::Response::SharedPtr response)
{    
    (void)response; // Is empty
    GenericCommanderLogic::send_notify_setpoints_stop(request->remain_valid_millisecs);
}


int 
GenericCommander::m_get_controller_parameter()
{
    rclcpp::Parameter param;
    if (m_node_parameters_interface->get_parameter(
            "stabilizer.controller",
            param))
    {
        int controller = param.as_int();
        return controller;
    }
    return -1;
}