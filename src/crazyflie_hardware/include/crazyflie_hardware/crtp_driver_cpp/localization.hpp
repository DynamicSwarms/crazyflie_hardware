

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/localization_logic.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

#include "object_tracker_interfaces/srv/add_tracker_object.hpp"
#include "object_tracker_interfaces/srv/remove_tracker_object.hpp"


#include "broadcaster_interfaces/srv/posi_pose_broadcast_object.hpp"

class Localization : public LocalizationLogic {
public:
    Localization(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<CrtpLink> link,
        std::string tf_name);
public: 

    bool stop_external_tracking();
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

    bool remove_from_tracker();
    bool remove_from_broadcaster();

private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> m_graph_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> m_services_interface;
    rclcpp::Logger m_logger;

    std::string m_tf_name;

    bool m_is_beeing_tracked; 
    bool m_is_beeing_broadcasted;

    int m_channel;
    int m_datarate;

    rclcpp::CallbackGroup::SharedPtr m_callback_group; 

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_console_publisher;
};
