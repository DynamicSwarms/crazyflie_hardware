#include "crazyflie_hardware_cpp/crtp_driver_cpp/parameters.hpp"
using std::placeholders::_1;

Parameters::Parameters(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link)
    : ParametersLogic(link, std::string("mein_pfad"))
    , node(node)
{
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    downdload_toc_sub = node->create_subscription<std_msgs::msg::Empty>(
                "~/download_parameters_toc", 
                10,
                std::bind(&Parameters::download_toc_callback, this, _1),
                sub_opt);

    get_toc_info_sub = node->create_subscription<std_msgs::msg::Empty>(
                "~/get_parameters_toc_info", 
                10,
                std::bind(&Parameters::get_toc_info_callback, this, _1),
                sub_opt);
    RCLCPP_WARN(node->get_logger(), "Parameters  initialized");

};

void Parameters::download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    ParametersLogic::send_download_toc_items();
}

void Parameters::get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    auto [nbr_of_items, crc] = ParametersLogic::send_get_toc_info();
    RCLCPP_WARN(node->get_logger(), "%d, %X", nbr_of_items, crc);

}