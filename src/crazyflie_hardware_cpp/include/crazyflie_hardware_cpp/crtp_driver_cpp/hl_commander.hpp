

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/hl_commander_logic.hpp"
#include "crazyflie_interfaces/msg/takeoff.hpp"
#include "crazyflie_interfaces/msg/land.hpp"

class HighLevelCommanderDriver : public HighLevelCommanderLogic {
public:
    HighLevelCommanderDriver(std::shared_ptr<rclcpp::Node> node, CrtpLink * link);
private: 

    void land_callback(const crazyflie_interfaces::msg::Land::SharedPtr msg);
    void takeoff_callback(const crazyflie_interfaces::msg::Takeoff::SharedPtr msg);
    

private: 
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<crazyflie_interfaces::msg::Land>::SharedPtr land_sub;
    rclcpp::Subscription<crazyflie_interfaces::msg::Takeoff>::SharedPtr takeoff_sub;
};