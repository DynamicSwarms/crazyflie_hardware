#include <cstdio>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "crtp_cpp/packer/toc_packer.hpp"

#include "crtp_cpp/logic/toc_logic.hpp"


#include "crtp_cpp/logic/generic_commander_logic.hpp"
#include "crtp_cpp/logic/hl_commander_logic.hpp"

#include "crtp_cpp/logic/console_logic.hpp"
//#include "crtp_cpp/logic/hl_commander_logic.hpp"
//#include "crtp_cpp/logic/link_layer_logic.hpp"


#include "crazyflie_hardware_cpp/crtp_driver_cpp/hl_commander.hpp"
#include "crazyflie_hardware_cpp/crtp_link_ros.hpp"




class Commander
{
  public: 
    Commander(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int channel) 
      : node(node)
      , link(node, channel, std::make_tuple(10), 250)
      , hl_commander(node, &link)
    {

    } 

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  RosLink link; 
  HighLevelCommanderDriver hl_commander;
};

class CrazyflieNode : public rclcpp_lifecycle::LifecycleNode
{
  


  public:
    CrazyflieNode(const rclcpp::NodeOptions & options)
      : rclcpp_lifecycle::LifecycleNode("crazyflie", options)
    {
      this->declare_parameter("channel", 80); // Make readonly
      channel = this->get_parameter("channel").as_int();

      timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CrazyflieNode::timer_callback, this));

    }
    void init()
    {
      //!!! Makeshared not in constructor
      auto node_ptr = this->shared_from_this();
      commander = std::make_unique<Commander>(node_ptr, channel);
    }
    void timer_callback()
    {   
      // HighLevelCommanderLogic logic(&link);
      // logic.send_stop(5);
        
    }


    /**
     * Lifecycle callbacks.
    */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
    {
    this->init();
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state)
    {
    LifecycleNode::on_activate(state);
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state)
    {
    LifecycleNode::on_deactivate(state);
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state)
    {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }




    rclcpp::TimerBase::SharedPtr timer; 
    std::unique_ptr<Commander> commander;
    int channel; 

};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  printf("hello world crazyflie_hardware_cpp package\n");


  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<CrazyflieNode>(options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
