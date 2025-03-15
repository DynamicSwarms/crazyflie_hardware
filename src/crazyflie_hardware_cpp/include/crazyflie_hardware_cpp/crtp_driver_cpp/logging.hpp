

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/logging_logic.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "crazyflie_interfaces/msg/generic_log_data.hpp"
#include "crazyflie_interfaces/msg/pose_stamped_array.hpp"

#include "std_msgs/msg/empty.hpp"

class Logging : public LoggingLogic {
public:
    Logging(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link);

    void initialize_logging();

    void start_logging_pm();
    void start_logging_pose();

private: 

    void download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg);

    void crtp_response_callback(const CrtpPacket&  packet) override; 

private: 
    

    rclcpp::CallbackGroup::SharedPtr callback_group; 

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr downdload_toc_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr get_toc_info_sub;

    bool log_pm;
    bool log_pose;
    rclcpp::Publisher<crazyflie_interfaces::msg::GenericLogData>::SharedPtr log_pm_pub;
    rclcpp::Publisher<crazyflie_interfaces::msg::PoseStampedArray>::SharedPtr log_pose_pub;

};  