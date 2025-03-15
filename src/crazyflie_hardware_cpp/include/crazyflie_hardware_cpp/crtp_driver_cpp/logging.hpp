

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/logging_logic.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "std_msgs/msg/empty.hpp"

class Logging : public LoggingLogic {
public:
    Logging(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link);
private: 

    void download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg);

    void initialize_logging();
    

private: 
    rclcpp::CallbackGroup::SharedPtr callback_group; 

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr downdload_toc_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr get_toc_info_sub;
};