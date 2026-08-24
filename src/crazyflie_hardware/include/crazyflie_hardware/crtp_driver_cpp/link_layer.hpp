#include "rclcpp/rclcpp.hpp"

#include "crtp_cpp/logic/link_layer_logic.hpp"


class LinkLayer : public LinkLayerLogic {
public:
    LinkLayer(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<CrtpLink> link);
private:
    rclcpp::Logger m_logger;
};