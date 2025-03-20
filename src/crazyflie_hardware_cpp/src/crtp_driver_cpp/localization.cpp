#include "crazyflie_hardware_cpp/crtp_driver_cpp/localization.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

Localization::Localization(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink *link, std::string tf_name)
    : LocalizationLogic(link)
    , node(node)
    , tf_name(tf_name)
    , is_beeing_tracked(false)
    , is_beeing_broadcasted(false)
    , channel_(80)
    , data_rate_(2)
{
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    RCLCPP_DEBUG(node->get_logger(), "Localization initialized");
}

bool Localization::stop_external_tracking() 
{
    if (is_beeing_tracked) remove_from_tracker();
    if (is_beeing_broadcasted) remove_from_broadcaster();
    return true;
}

bool Localization::start_external_tracking(int marker_configuration_index,
                                           int dynamics_configuration_index,
                                           double max_initial_deviation,
                                           std::vector<double> initial_position,
                                           int channel,
                                           int datarate)
{
    is_beeing_tracked = add_to_tracker(marker_configuration_index, dynamics_configuration_index, max_initial_deviation, initial_position);
    if (is_beeing_tracked)
    {
        is_beeing_broadcasted = add_to_broadcaster(channel, datarate);
        if (is_beeing_broadcasted) 
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
    rclcpp::Client<object_tracker_interfaces::srv::AddTrackerObject>::SharedPtr client =
        node->create_client<object_tracker_interfaces::srv::AddTrackerObject>(
            "/tracker/add_object",
            rclcpp::QoS(rclcpp::KeepLast(1)).get_rmw_qos_profile(),
            callback_group);

    if (!client->wait_for_service(1s))
    {
        RCLCPP_WARN(node->get_logger(), "Tracking Service not available!");
        return false;
    }

    auto request = std::make_shared<object_tracker_interfaces::srv::AddTrackerObject::Request>();
    request->tf_name.data = tf_name;
    request->marker_configuration_idx = marker_configuration_index;
    request->dynamics_configuration_idx = dynamics_configuration_index;
    request->max_initial_deviation = max_initial_deviation;
    request->initial_pose.position.x = initial_position[0];
    request->initial_pose.position.y = initial_position[1];
    request->initial_pose.position.z = initial_position[2];
    auto result = client->async_send_request(request);

    RCLCPP_WARN(node->get_logger(), "wait for service success");


    auto status = result.wait_for(3s); // not spinning here!
    if (status == std::future_status::ready)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call success!");
        auto res = result.get();
        if (res->success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add to tracker success!");
            return true;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add to tracker failed!");
        }
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call timed out!");
    }

    return false;
}

bool Localization::remove_from_tracker() 
{
    rclcpp::Client<object_tracker_interfaces::srv::RemoveTrackerObject>::SharedPtr client =
        node->create_client<object_tracker_interfaces::srv::RemoveTrackerObject>(
            "/tracker/remove_object",
            rclcpp::QoS(rclcpp::KeepLast(1)).get_rmw_qos_profile(),
            callback_group);
    
    if (!client->wait_for_service(50ms))
    {
        RCLCPP_WARN(node->get_logger(), "Tracking Service not available!");
        return false;
    }
    
    auto request = std::make_shared<object_tracker_interfaces::srv::RemoveTrackerObject::Request>();
    request->tf_name.data = tf_name;
    
    auto result = client->async_send_request(request);
    auto status = result.wait_for(50ms);
    return status == std::future_status::ready;
}

bool Localization::add_to_broadcaster(int channel, int datarate)
{
    channel_ = channel;
    data_rate_ = datarate;
    rclcpp::Client<broadcaster_interfaces::srv::PosiPoseBroadcastObject>::SharedPtr client =
        node->create_client<broadcaster_interfaces::srv::PosiPoseBroadcastObject>(
            "/add_posi_pose_object",
            rclcpp::QoS(rclcpp::KeepLast(1)).get_rmw_qos_profile(),
            callback_group);

    if (!client->wait_for_service(1s))
    {
        RCLCPP_WARN(node->get_logger(), "Broadcast Service not available!");
        return false;
    }

    auto request = std::make_shared<broadcaster_interfaces::srv::PosiPoseBroadcastObject::Request>();
    request->channel = channel;
    request->data_rate = datarate;
    request->tf_frame_id = tf_name;

    auto result = client->async_send_request(request);

    RCLCPP_WARN(node->get_logger(), "wait for service success");

    using namespace std::chrono_literals;

    auto status = result.wait_for(3s); // not spinning here!
    if (status == std::future_status::ready)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call success!");
        auto res = result.get();
        if (res->success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Broadcast success!");
            return true;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add to Broadcast failed!");
        }
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call timed out!");
    }

    return false;
}

bool Localization::remove_from_broadcaster()
{
    rclcpp::Client<broadcaster_interfaces::srv::PosiPoseBroadcastObject>::SharedPtr client =
        node->create_client<broadcaster_interfaces::srv::PosiPoseBroadcastObject>(
            "/remove_posi_pose_object",
            rclcpp::QoS(rclcpp::KeepLast(1)).get_rmw_qos_profile(),
            callback_group);

    if (!client->wait_for_service(50ms))
    {
        RCLCPP_WARN(node->get_logger(), "Tracking Service not available!");
        return false;
    }
    
    auto request = std::make_shared<broadcaster_interfaces::srv::PosiPoseBroadcastObject::Request>();
    request->channel = channel_;
    request->data_rate = data_rate_;
    request->tf_frame_id = tf_name;
    
    auto result = client->async_send_request(request);
    auto status = result.wait_for(50ms);
    return status == std::future_status::ready;
}
