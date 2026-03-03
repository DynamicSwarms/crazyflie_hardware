#include "rclcpp/rclcpp.hpp"

#include "crtp_cpp/logic/platform_logic.hpp"


class Platform : public PlatformLogic {
public:
    Platform(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        CrtpLink * link);
private:
    rclcpp::Logger m_logger;
};