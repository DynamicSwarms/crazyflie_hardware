#include "crazyflie_hardware/crtp_driver_cpp/localization.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

Localization::Localization(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<CrtpLink> link,
    std::string tf_name)
    : LocalizationLogic(link)
    , m_base_interface(node_base_interface)
    , m_graph_interface(node_graph_interface)
    , m_services_interface(node_services_interface)
    , m_logger(node_logging_interface->get_logger().get_child("Localization"))
    , m_tf_name(tf_name)
    , m_is_beeing_tracked(false)
    , m_is_beeing_broadcasted(false)
    , m_channel(80)
    , m_datarate(2)
    , m_callback_group(m_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))

{
    RCLCPP_DEBUG(m_logger, "Localization initialized");
}

bool Localization::stop_external_tracking() 
{
    bool ret = true; 
    if (m_is_beeing_tracked) {
        if (!remove_from_tracker()) ret = false; 
        else m_is_beeing_tracked = false;
    }
    if (m_is_beeing_broadcasted)
    {
        if (!remove_from_broadcaster()) ret = false;
        else m_is_beeing_broadcasted = false;
    } 
    return ret;
}

bool Localization::start_external_tracking(int marker_configuration_index,
                                           int dynamics_configuration_index,
                                           double max_initial_deviation,
                                           std::vector<double> initial_position,
                                           int channel,
                                           int datarate)
{
    m_is_beeing_tracked = add_to_tracker(marker_configuration_index, dynamics_configuration_index, max_initial_deviation, initial_position);
    if (m_is_beeing_tracked)
    {
        m_is_beeing_broadcasted = add_to_broadcaster(channel, datarate);
        if (m_is_beeing_broadcasted) 
        {
            return true;
        }
    }
    return false;
}

bool Localization::add_to_tracker(
    int marker_configuration_index,
    int dynamics_configuration_index,
    double max_initial_deviation,
    std::vector<double> initial_position)
{
    rclcpp::Client<object_tracker_interfaces::srv::AddTrackerObject>::SharedPtr client;
    client = rclcpp::create_client<object_tracker_interfaces::srv::AddTrackerObject>(
        m_base_interface,
        m_graph_interface,
        m_services_interface,
        "/tracker/add_object",
        rclcpp::ServicesQoS().keep_last(1),
        m_callback_group);
    
    if (!client->wait_for_service(1s))
    {
        RCLCPP_WARN(m_logger, "Tracking Service not available!");
        return false;
    }

    auto request = std::make_shared<object_tracker_interfaces::srv::AddTrackerObject::Request>();
    request->tf_name.data = m_tf_name;
    request->marker_configuration_idx = marker_configuration_index;
    request->dynamics_configuration_idx = dynamics_configuration_index;
    request->max_initial_deviation = max_initial_deviation;
    request->initial_pose.position.x = initial_position[0];
    request->initial_pose.position.y = initial_position[1];
    request->initial_pose.position.z = initial_position[2];
    request->initialization_timeout = 2.0; // seconds

    auto result = client->async_send_request(request);

    auto status = result.wait_for(3s); // not spinning here!
    if (status == std::future_status::ready)
    {
        RCLCPP_DEBUG(m_logger, "Service call success!");
        auto res = result.get();
        if (res->success)
        {
            RCLCPP_DEBUG(m_logger, "Add to tracker success!");
            return true;
        }
        else
        {
            RCLCPP_INFO(m_logger, "Add to tracker failed!");
        }
    }
    else
    {
        RCLCPP_DEBUG(m_logger, "Service call timed out!");
    }

    return false;
}



bool Localization::add_to_broadcaster(int channel, int datarate)
{
    m_channel = channel;
    m_datarate = datarate;
    rclcpp::Client<broadcaster_interfaces::srv::PosiPoseBroadcastObject>::SharedPtr client;
    client = rclcpp::create_client<broadcaster_interfaces::srv::PosiPoseBroadcastObject>(
        m_base_interface,
        m_graph_interface,
        m_services_interface,
        "/add_posi_pose_object",
        rclcpp::ServicesQoS().keep_last(1),
        m_callback_group);
    
    if (!client->wait_for_service(1s))
    {
        RCLCPP_WARN(m_logger, "Broadcast Service not available!");
        return false;
    }

    auto request = std::make_shared<broadcaster_interfaces::srv::PosiPoseBroadcastObject::Request>();
    request->channel = m_channel;
    request->data_rate = m_datarate;
    request->tf_frame_id = m_tf_name;

    auto result = client->async_send_request(request);

    auto status = result.wait_for(3s); // not spinning here!
    if (status == std::future_status::ready)
    {
        auto res = result.get();
        if (res->success)
        {
            RCLCPP_DEBUG(m_logger, "Broadcast success!");
            return true;
        }
        else
        {
            RCLCPP_INFO(m_logger, "Add to Broadcast failed!");
        }
    }
    else
    {
        RCLCPP_DEBUG(m_logger, "Service call timed out!");
    }

    return false;
}

bool Localization::remove_from_tracker() 
{
    rclcpp::Client<object_tracker_interfaces::srv::RemoveTrackerObject>::SharedPtr client;
    client = rclcpp::create_client<object_tracker_interfaces::srv::RemoveTrackerObject>(
        m_base_interface,
        m_graph_interface,
        m_services_interface,            
        "/tracker/remove_object",
        rclcpp::ServicesQoS().keep_last(1),
        m_callback_group);
    
    if (!client->wait_for_service(100ms))
    {
        RCLCPP_WARN(m_logger, "Tracking Service not available!");
        return false;
    }
    
    auto request = std::make_shared<object_tracker_interfaces::srv::RemoveTrackerObject::Request>();
    request->tf_name.data = m_tf_name;
    
    auto result = client->async_send_request(request);
    auto status = result.wait_for(100ms);

    bool ret = status == std::future_status::ready;
    if (!ret) {
        RCLCPP_WARN(m_logger, "Tracker didnt respond in time!");
    }
    return ret;
}

bool Localization::remove_from_broadcaster()
{
    rclcpp::Client<broadcaster_interfaces::srv::PosiPoseBroadcastObject>::SharedPtr client;
    client = rclcpp::create_client<broadcaster_interfaces::srv::PosiPoseBroadcastObject>(
        m_base_interface,
        m_graph_interface,
        m_services_interface,
        "/remove_posi_pose_object",
        rclcpp::ServicesQoS().keep_last(1),
        m_callback_group);

    if (!client->wait_for_service(100ms))
    {
        RCLCPP_WARN(m_logger, "Broadcast Service not available!");
        return false;
    }
    
    auto request = std::make_shared<broadcaster_interfaces::srv::PosiPoseBroadcastObject::Request>();
    request->channel = m_channel;
    request->data_rate = m_datarate;
    request->tf_frame_id = m_tf_name;
    
    auto result = client->async_send_request(request);
    auto status = result.wait_for(100ms);
    bool ret = status == std::future_status::ready;
    if (!ret) {
        RCLCPP_WARN(m_logger, "Broadcaster didnt respond in time!");
    }
    return ret;
}
