

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/hl_commander_logic.hpp"
#include "crazyflie_interfaces/msg/takeoff.hpp"
#include "crazyflie_interfaces/msg/land.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class HighLevelCommander : public HighLevelCommanderLogic {
public:
    HighLevelCommander(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link);
private: 

    void land_callback(const crazyflie_interfaces::msg::Land::SharedPtr msg);
    void takeoff_callback(const crazyflie_interfaces::msg::Takeoff::SharedPtr msg);
    

private: 
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
    rclcpp::Subscription<crazyflie_interfaces::msg::Land>::SharedPtr land_sub;
    rclcpp::Subscription<crazyflie_interfaces::msg::Takeoff>::SharedPtr takeoff_sub;
};