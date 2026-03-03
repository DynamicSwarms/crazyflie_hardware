

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/console_logic.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

class Console : public ConsoleLogic
{
public:
    Console(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        CrtpLink *link);

private:
    void console_message(const std::string);
    void crtp_response_callback(const CrtpPacket &packet) override;

private:
    rclcpp::Logger m_logger;

    rclcpp::CallbackGroup::SharedPtr callback_group;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr console_publisher;
};