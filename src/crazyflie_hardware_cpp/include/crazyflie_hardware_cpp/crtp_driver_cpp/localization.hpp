

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/localization_logic.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

#include "object_tracker_interfaces/srv/add_tracker_object.hpp"
#include "object_tracker_interfaces/srv/remove_tracker_object.hpp"


#include "broadcaster_interfaces/srv/posi_pose_broadcast_object.hpp"

class Localization : public LocalizationLogic {
public:
    Localization(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link, std::string tf_name);
public: 

    void console_message(const std::string);

    bool start_external_tracking(   int marker_configuration_index,
                                    int dynamics_configuration_index, 
                                    double max_initial_deviation, 
                                    std::vector<double> initial_position,
                                    int channel,
                                    int datarate);

private: 
    bool add_to_tracker(
        int marker_configuration_index,
        int dynamics_configuration_index, 
        double max_initial_deviation,
        std::vector<double> initial_position);
    bool add_to_broadcaster(int channel, int datarate);

    
    std::string tf_name;

    rclcpp::CallbackGroup::SharedPtr callback_group; 

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr console_publisher;
};