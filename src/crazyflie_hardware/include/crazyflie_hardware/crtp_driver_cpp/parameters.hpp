

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/parameters_logic.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "std_msgs/msg/empty.hpp"

class Parameters : public ParametersLogic
{
public:
    Parameters(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        CrtpLink *link);
    void initialize_parameters();

private:
    void m_download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void m_get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg);

    rcl_interfaces::msg::SetParametersResult m_set_parameter_callback(const std::vector<rclcpp::Parameter> &parameters);

private:
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> m_parameters_interface;
    rclcpp::Logger m_logger;

    rclcpp::CallbackGroup::SharedPtr m_callback_group;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_downdload_toc_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_get_toc_info_sub;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;
};