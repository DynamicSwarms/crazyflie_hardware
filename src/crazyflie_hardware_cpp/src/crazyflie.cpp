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

#include "crazyflie_hardware_cpp/crtp_driver_cpp/hl_commander.hpp"
#include "crazyflie_hardware_cpp/crtp_driver_cpp/generic_commander.hpp"
#include "crazyflie_hardware_cpp/crtp_driver_cpp/parameters.hpp"
#include "crazyflie_hardware_cpp/crtp_driver_cpp/console.hpp"
#include "crazyflie_hardware_cpp/crtp_driver_cpp/localization.hpp"



#include "crazyflie_hardware_cpp/crtp_link_ros.hpp"



//#include "crtp_cpp/logic/hl_commander_logic.hpp"
//#include "crtp_cpp/logic/link_layer_logic.hpp"


class Commander
{
  public: 
    Commander(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int channel, std::array<uint8_t, 5> address, int datarate) 
      : node(node)
      , link(node, channel, address, datarate)
      , console(node, &link)
      , hl_commander(node, &link)
      , generic_commander(node, &link)
      , parameters(node, &link)
      , localization(node, &link, get_tf_name(address))
    {
      RCLCPP_WARN(node->get_logger(), "Setting default Parameters");


      std::map<std::string, rclcpp::Parameter> default_params;
      node->get_node_parameters_interface()->get_parameters_by_prefix("default_firmware_params", default_params);
      for (const auto & param : default_params) 
      {
          rclcpp::Parameter set_param(param.first, param.second.get_parameter_value());
          node->set_parameter(set_param);          
              
      }
      rclcpp::Parameter set_param("kalman.resetEstimation",1);
      node->set_parameter(set_param);    

      RCLCPP_WARN(node->get_logger(), "Setting up tracking services.");
      int id =  node->get_parameter("id").as_int();
      std::vector<double> initial_position = node->get_parameter("initial_position").as_double_array();
      bool send_external_position = node->get_parameter("send_external_position").as_bool();
      bool send_external_pose = node->get_parameter("send_external_pose").as_bool();
      double max_initial_deviation = node->get_parameter("max_initial_deviation").as_double();
      int marker_configuration_index = node->get_parameter("marker_configuration_index").as_int();
      int dynamics_configuration_index = node->get_parameter("dynamics_configuration_index").as_int();
     
      localization.start_external_tracking(marker_configuration_index, dynamics_configuration_index, max_initial_deviation, initial_position, channel, datarate);
    } 
  
private:
    std::string get_tf_name(const std::array<uint8_t, 5>& address) {
        std::stringstream ss;
        ss << "cf" << static_cast<int>(address[4]); // Assuming the ID is in the last element of the address
        return ss.str();
    }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  RosLink link; 
  Console console;
  HighLevelCommander hl_commander;
  GenericCommander generic_commander;
  Parameters parameters;
  Localization localization;
};

class CrazyflieNode : public rclcpp_lifecycle::LifecycleNode
{
  


  public:
    CrazyflieNode(const rclcpp::NodeOptions & options)
      : rclcpp_lifecycle::LifecycleNode("crazyflie", options)
    {
      rcl_interfaces::msg::ParameterDescriptor readonly_descriptor;
      readonly_descriptor.read_only = true;

      this->declare_parameter("id", 0xE7, readonly_descriptor); 
      this->declare_parameter("channel", 80, readonly_descriptor);
      std::vector<double> default_initial_position = {0.0,0.0,0.0};
      this->declare_parameter("initial_position", default_initial_position, readonly_descriptor); 
      this->declare_parameter("datarate", 2, readonly_descriptor);

      // Parameters from crazyflie types.yaml
      this->declare_parameter("send_external_position", false, readonly_descriptor);
      this->declare_parameter("send_external_pose", false, readonly_descriptor); 
      this->declare_parameter("max_initial_deviation", 1.0, readonly_descriptor);
      this->declare_parameter("marker_configuration_index", 4, readonly_descriptor); 
      this->declare_parameter("dynamics_configuration_index",0, readonly_descriptor);


      
      id =  get_parameter("id").as_int();
      channel = get_parameter("channel").as_int();
      initial_position = get_parameter("initial_position").as_double_array();
      datarate = get_parameter("datarate").as_int();
      send_external_position = get_parameter("send_external_position").as_bool();
      send_external_pose = get_parameter("send_external_pose").as_bool();
      max_initial_deviation = get_parameter("max_initial_deviation").as_double();
      marker_configuration_index = get_parameter("marker_configuration_index").as_int();
      dynamics_configuration_index = get_parameter("dynamics_configuration_index").as_int();


      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;
      descriptor.read_only = true;
      for (const auto & pair : this->get_node_parameters_interface()->get_parameter_overrides()) {
        size_t dot = pair.first.find('.');
        if (dot != std::string::npos) {
            std::string ns = pair.first.substr(0, dot);
            std::string name = pair.first.substr(dot + 1);

            if (ns == "default_firmware_params") {
              this->declare_parameter(
              pair.first,
              pair.second,
              descriptor,
              true);
            }
        }
      }

      address = {0xE7, 0xE7, 0xE7, 0xE7, id};
      
      timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CrazyflieNode::timer_callback, this));

    }
    void init()
    {

      

      //!!! Makeshared not in constructor
      auto node_ptr = this->shared_from_this();
      commander = std::make_unique<Commander>(node_ptr, channel, address, datarate);
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
    RCLCPP_INFO(get_logger(), "configuring");

    this->init();
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


    std::array<uint8_t, 5> address;
    
    uint8_t id;
    int channel;
    std::vector<double> initial_position;
    int datarate;
    bool send_external_position;
    bool send_external_pose;
    double max_initial_deviation;
    int marker_configuration_index;
    int dynamics_configuration_index;

};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  printf("hello world crazyflie_hardware_cpp package\n");


  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<CrazyflieNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
