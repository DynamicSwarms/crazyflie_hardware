#pragma once

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_interfaces/msg/log_data_generic.hpp"

class LogBlock {

    public:
    LogBlock(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        rclcpp::Logger logger,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
        std::shared_ptr<rclcpp::CallbackGroup> callback_group,
        const std::string block_name
    );

    void m_publish_log_data(std::vector<double>& data);

private:

private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_base_interface;
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_timers_interface;
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    std::string m_block_name;   
    std::shared_ptr<rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>> m_log_data_publisher;
};