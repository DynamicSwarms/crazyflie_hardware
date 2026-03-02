#include "crazyflie_hardware/crtp_driver_cpp/logblock.hpp"

LogBlock::LogBlock(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::CallbackGroup> callback_group,
    const std::string name) 
    : m_base_interface(node_base_interface)
    , m_logging_interface(node_logging_interface)
    , m_timers_interface(node_timers_interface)
    , m_callback_group(callback_group)
    , m_block_name(name)
{   
    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = callback_group;
    m_log_data_publisher = rclcpp::create_publisher<crazyflie_interfaces::msg::LogDataGeneric>(
        node_topics_interface,
        "~/" + name,
        rclcpp::QoS(10),
        publisher_options
    );
}

void LogBlock::m_publish_log_data(std::vector<double>& data)
{
    auto log_msg = crazyflie_interfaces::msg::LogDataGeneric();
    
    for (size_t i = 0; i < data.size(); i++) {
        log_msg.values.push_back(data[i]);
    }

    m_log_data_publisher->publish(log_msg);
}