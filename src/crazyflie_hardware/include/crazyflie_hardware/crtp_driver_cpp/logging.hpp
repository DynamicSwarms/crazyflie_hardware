#pragma once

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/logging_logic.hpp"

#include "crazyflie_hardware/crtp_driver_cpp/logblock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/pose_named_array.hpp"

#include "crazyflie_interfaces/srv/add_logging.hpp"
#include "crazyflie_interfaces/srv/remove_logging.hpp"

#include "std_msgs/msg/empty.hpp"

class Logging : public LoggingLogic {
public:
    Logging(
        std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        CrtpLink * link);

    void initialize_logging();

    void start_logging_pm();
    void start_logging_pose();

private: 

    void download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg);

    void m_add_log_block_service(
        const std::shared_ptr<crazyflie_interfaces::srv::AddLogging::Request> request,
        std::shared_ptr<crazyflie_interfaces::srv::AddLogging::Response> response
    );

    void m_remove_log_block_service(
        const std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging::Request> request,
        std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging::Response> response
    );

    bool m_create_log_block(
        const std::string &block_name,
        const std::vector<std::string> &variables);

    void crtp_response_callback(const CrtpPacket&  packet) override; 

    void m_start_logging_block(int block_id, int period_ms_d10)
    {
        LoggingLogic::start_block(block_id, period_ms_d10);
    }
    void m_stop_logging_block(int block_id)
    {
        LoggingLogic::stop_block(block_id);
    }


private: 
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> m_node;
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> m_topics_interface;
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_timers_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_clock_interface;


    rclcpp::CallbackGroup::SharedPtr m_callback_group; 

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr downdload_toc_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr get_toc_info_sub;
    
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::AddLogging>> m_add_log_block_server;
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::RemoveLogging>> m_remove_log_block_server;

    bool log_state;
    bool log_pose;
    rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr log_state_pub;
    rclcpp::Publisher<crazyflie_interfaces::msg::PoseNamedArray>::SharedPtr log_pose_pub;

    uint8_t next_log_block_id = 2;
    std::map<int, std::shared_ptr<LogBlock>> m_log_blocks;
    std::map<std::string, int> m_log_block_ids;
};  