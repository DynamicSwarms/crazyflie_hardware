#include "crazyflie_hardware/crtp_driver_cpp/hl_commander.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

HighLevelCommander::HighLevelCommander(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    CrtpLink * link)
    : HighLevelCommanderLogic(link)
    , m_logger(node_logging_interface->get_logger().get_child("HighLevelCommander"))
    , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    (void)node_topics_interface;

    m_land_service = rclcpp::create_service<crazyflie_interfaces::srv::Land>(
        node_base_interface,
        node_services_interface,
        "~/land",
        std::bind(&HighLevelCommander::land_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );

    m_takeoff_service = rclcpp::create_service<crazyflie_interfaces::srv::Takeoff>(
        node_base_interface,
        node_services_interface,
        "~/takeoff",
        std::bind(&HighLevelCommander::takeoff_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );

    m_goto_service = rclcpp::create_service<crazyflie_interfaces::srv::GoTo>(
        node_base_interface,
        node_services_interface,
        "~/go_to",
        std::bind(&HighLevelCommander::goto_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );


    RCLCPP_DEBUG(m_logger, "High Level Commander initialized");
};

void HighLevelCommander::land_service(
    const crazyflie_interfaces::srv::Land::Request::SharedPtr request, 
    crazyflie_interfaces::srv::Land::Response::SharedPtr response)
{
    (void)response;
    HighLevelCommanderLogic::send_land(request->height, (double)(request->duration.sec + request->duration.nanosec * 1e-9), request->group_mask, request->yaw);
}

void HighLevelCommander::takeoff_service(
    const crazyflie_interfaces::srv::Takeoff::Request::SharedPtr request, 
    crazyflie_interfaces::srv::Takeoff::Response::SharedPtr response)
{
    (void)response;
    HighLevelCommanderLogic::send_takeoff(request->height, (double)(request->duration.sec + request->duration.nanosec * 1e-9), request->group_mask, request->yaw);
}

void HighLevelCommander::goto_service(
    const crazyflie_interfaces::srv::GoTo::Request::SharedPtr request, 
    crazyflie_interfaces::srv::GoTo::Response::SharedPtr response)
{
    (void)response;
    HighLevelCommanderLogic::send_go_to(request->goal.x, request->goal.y, request->goal.z, request->yaw, (double)(request->duration.sec + request->duration.nanosec * 1e-9), request->relative, request->group_mask);
}