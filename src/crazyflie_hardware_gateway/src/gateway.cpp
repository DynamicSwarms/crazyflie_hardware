#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rclcpp_components/node_factory.hpp"

#include "ament_index_cpp/get_resource.hpp"
#include "rcpputils/split.hpp"
#include "class_loader/class_loader.hpp"


#include "crazyflie_interfaces/srv/add_crazyflie.hpp"
#include "crazyflie_interfaces/srv/remove_crazyflie.hpp"

#include <memory>
#include <filesystem>

#include "signal.h"


std::atomic_bool sigint_received(false);
std::atomic_bool gateway_shutdown_done(false);

class GatewayException : public std::runtime_error
{
public:
  explicit GatewayException(const std::string & error_desc)
  : std::runtime_error(error_desc) {}
};

struct DedicatedExecutorWrapper
  {
    std::shared_ptr<rclcpp::Executor> executor;
    std::thread thread;
    std::atomic_bool thread_initialized;

    /// Constructor for the wrapper.
    /// This is necessary as atomic variables don't have copy/move operators
    /// implemented so this structure is not copyable/movable by default
    explicit DedicatedExecutorWrapper(std::shared_ptr<rclcpp::Executor> exec)
    : executor(exec),
      thread_initialized(false)
    {
    }
  };

class CrazyflieGateway : public rclcpp::Node
{
public:
  CrazyflieGateway(
    std::weak_ptr<rclcpp::Executor> executor,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("crazyflie_hardware_gateway", options)
  , crazyflies_()
  , executor_(executor)
  {
    auto service_qos = rclcpp::ServicesQoS().keep_last(100); // This way it is possible to queue up multiple add requestst

    add_service_ = this->create_service<crazyflie_interfaces::srv::AddCrazyflie>(
      "~/add_crazyflie", std::bind(&CrazyflieGateway::handle_add_crazyflie, this, std::placeholders::_1, std::placeholders::_2),
      service_qos);

    remove_service_ = this->create_service<crazyflie_interfaces::srv::RemoveCrazyflie>(
      "~/remove_crazyflie", std::bind(&CrazyflieGateway::handle_remove_crazyflie, this, std::placeholders::_1, std::placeholders::_2),
      service_qos);
    
    cleanup_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CrazyflieGateway::cleanup_callback, this));
  
    factory_ = create_component_factory("crazyflie_hardware", "CrazyflieNode");

    RCLCPP_INFO(get_logger(), "Crazyflie Gateway ready.");
  }

private: 
  std::pair<bool, std::string> add_crazyflie(int id, 
    int channel, 
    const geometry_msgs::msg::Pose & initial_pose,
    const std::string & type)
  {
    RCLCPP_INFO(get_logger(), "Adding Crazyflie with Channel: %d, Id: %d", channel, id);
    std::pair<uint8_t, uint8_t> crazyflie_key = {id, channel};

    if (crazyflies_.count(crazyflie_key))
    {
      return std::make_pair(false, "Crazyflie with id '" + std::to_string(id) + "' already exists.");
    }

    auto options = create_node_options(id, channel, initial_pose, type);
    try {
      auto node = factory_->create_node_instance(options);
      auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      exec->add_node(node.get_node_base_interface());

      auto entry = crazyflies_.emplace(std::make_pair(crazyflie_key, std::make_pair(node, exec)));

      DedicatedExecutorWrapper & wrapper = entry.first->second.second;
      wrapper.executor = exec;

      auto & thread_initialized = wrapper.thread_initialized;
      wrapper.thread = std::thread(
        [exec, &thread_initialized ]() {
          thread_initialized = true;
          try {
            exec->spin();
          } catch (...) {}
        }
      );
      // // Downcast to lifecycle node interface
      auto lifecycle_node = std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node.get_node_instance());
      if (!lifecycle_node) {
         throw std::runtime_error("Failed to cast to LifecycleNodeInterface");
       }
      // // Transition to configure
       auto ret = lifecycle_node.get()->configure();    
       if (ret.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
       {
          remove_crazyflie(id, channel);
          return std::make_pair(false, "Configuration failed.");
       }

    } catch (const std::exception & ex) {
      // In the case that the component constructor throws an exception,
      // rethrow into the following catch block.
      throw GatewayException(
              "Component constructor threw an exception: " + std::string(ex.what()));
    } catch (...) {
      // In the case that the component constructor throws an exception,
      // rethrow into the following catch block.
      throw GatewayException("Component constructor threw an exception");
    }
    
    return std::make_pair(true, "Success!");
  }

  std::pair<bool, std::string> remove_crazyflie(int id, int channel)
  {
    RCLCPP_INFO(get_logger(), "Removing Crazyflie with Channel: %d, Id: %d", channel, id);
    std::pair<uint8_t, uint8_t> crazyflie_key = {id, channel};
    auto cf = crazyflies_.find(crazyflie_key);
    if (cf == crazyflies_.end()) {
      return std::make_pair(false,
        "Couldn't remove Crazyflie with Channel: " + std::to_string(channel) 
        + ", ID: " + std::to_string(id) + "; not in list.");
    }
    
    if (!cf->second.second.thread_initialized)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(1)); // This only happens when add and removed are called near simultaniously. 
    }
    cf->second.second.executor->cancel();
    cf->second.second.thread.join();
    crazyflies_.erase(cf);
    return std::make_pair(true, "Success!");
  }

private:
  void cleanup_callback()
  {
    for (auto& [key, entry] : crazyflies_ )
    {
      auto lifecycle_node = std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(entry.first.get_node_instance());
      if (lifecycle_node) 
      {
        if (lifecycle_node.get()->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
        {
          remove_crazyflie(key.first, key.second);
          return;
        }
      }
    }

    static bool shutdown_completed = false;
    if (sigint_received.load() && !shutdown_completed)
    {
        shutdown_completed = true;
        if (crazyflies_.empty()) gateway_shutdown_done.store(true);
        else RCLCPP_INFO(this->get_logger(), "Shutting down all crazyflies due to SIGINT.");
        std::vector<std::pair<uint8_t, uint8_t>> keys_to_remove;
        for (const auto& [key, entry] : crazyflies_) {
          keys_to_remove.push_back(key);
        }
        for (auto &key : keys_to_remove)
        {
          std::pair<bool, std::string> result = remove_crazyflie(key.first, key.second);
          if (!result.first) RCLCPP_WARN(this->get_logger(), "Failed to remove crazyflie with id %d: %s", key.first, result.second.c_str());
          else RCLCPP_INFO(this->get_logger(), "Successfully removed crazyflie with id %d.", key.first);
        }
        gateway_shutdown_done.store(true);
    }
  }

  void handle_add_crazyflie(const std::shared_ptr<crazyflie_interfaces::srv::AddCrazyflie::Request> request,
                            std::shared_ptr<crazyflie_interfaces::srv::AddCrazyflie::Response> response)
  {
    try {
      auto [id, channel] = uri_to_id_channel(request->uri);
      auto [success, msg] = add_crazyflie(id, channel, request->initial_pose, request->type);
      response->success = success;
      response->msg = msg;
    } catch (const GatewayException & ex) {
      response->success = false;
      response->msg = ex.what();
    }
  }

  void handle_remove_crazyflie(const std::shared_ptr<crazyflie_interfaces::srv::RemoveCrazyflie::Request> request,
                               std::shared_ptr<crazyflie_interfaces::srv::RemoveCrazyflie::Response> response)
  {
    try {
      auto [id, channel] = uri_to_id_channel(request->uri);
      auto [success, msg] = remove_crazyflie(id, channel);
      
      response->success = success;
      response->msg = msg;
    } catch (const GatewayException & ex) {
      response->success = false;
      response->msg = ex.what();
    }
  }

private: 
  std::pair<uint8_t, uint8_t> 
  uri_to_id_channel(const std::string & uri)
  {
    const std::string prefix = "radio://";
    if (uri.rfind(prefix, 0) != 0) {
        throw GatewayException("URI must start with 'radio://'");
    }

    try {
        const auto payload = uri.substr(prefix.length());
        const auto parts = rcpputils::split(payload, '/');
        if (parts.size() < 4) {
          throw GatewayException("URI must have format radio://<dongle>/<channel>/<datarate>/<address>");
        }

        const int channel = std::stoi(parts[1]);
        if (channel < 0 || channel > 255) {
          throw GatewayException("Channel out of range [0,255]");
        }

        const std::string & address = parts[3];
        if (address.size() < 2) {
          throw GatewayException("Address must contain at least one byte");
        }

        const std::string id_hex = address.substr(address.size() - 2);
        const int id = std::stoi(id_hex, nullptr, 16);
        if (id < 0 || id > 255) {
          throw GatewayException("ID out of range [0,255]");
        }

        return {static_cast<uint8_t>(id), static_cast<uint8_t>(channel)};
    } catch (...) {
        throw GatewayException("Failed to parse URI; expected radio://<dongle>/<channel>/<datarate>/<address> with hex address");
    }
  }

  rclcpp::NodeOptions
  create_node_options(int id, 
                      int channel, 
                      const geometry_msgs::msg::Pose & initial_pose,
                      const std::string & type)
  {


    std::vector<std::string> remap_rules;

    remap_rules.push_back("--ros-args");
    remap_rules.push_back("-r");
    remap_rules.push_back("__node:=cf" + std::to_string(id));

    auto add_parameter = [&](const std::string &name, const std::string &value) {
      remap_rules.push_back("-p");
      remap_rules.push_back(name + ":=" + value);
    };

    add_parameter("id", std::to_string(id));
    add_parameter("channel", std::to_string(channel));
    add_parameter("datarate", "2");

    std::ostringstream pos_stream;
    pos_stream << std::fixed << std::setprecision(1) << "[" 
              << initial_pose.position.x << "," 
              << initial_pose.position.y << "," 
              << initial_pose.position.z << "]";
    
    add_parameter("initial_position", pos_stream.str());

    add_parameter("send_external_position", get_parameter("crazyflieTypes." +  type + ".sendExternalPosition").as_bool() ? "True" : "False");
    add_parameter("send_external_pose", get_parameter("crazyflieTypes." +  type + ".sendExternalPose").as_bool() ? "True" : "False");
    
    add_parameter("max_initial_deviation", std::to_string(get_parameter("crazyflieTypes." +  type + ".maxInitialDeviation").as_double()));
    add_parameter("marker_configuration_index", std::to_string(get_parameter("crazyflieTypes." +  type + ".markerConfigurationIndex").as_int()));
    add_parameter("dynamics_configuration_index",std::to_string(get_parameter("crazyflieTypes." +  type + ".dynamicsConfigurationIndex").as_int()));
    
    remap_rules.push_back("--params-file");
    remap_rules.push_back(get_parameter("crazyflie_configuration_yaml").as_string());

    auto options = rclcpp::NodeOptions()
      .arguments(remap_rules);
      return options;
  }

  std::vector<std::pair<std::string, std::string>>
  get_component_resources(
    const std::string & package_name, const std::string & resource_index) const
  {
    auto result = ament_index_cpp::get_resource(resource_index, package_name);
    if (result.resourcePath == std::nullopt) {
      throw GatewayException("Could not find requested resource in ament index");
    }


    std::vector<std::pair<std::string, std::string>> resources;
    std::vector<std::string> lines = rcpputils::split(result.contents, '\n', true);
    for (const auto & line : lines) {
      std::vector<std::string> parts = rcpputils::split(line, ';');
      if (parts.size() != 2) {
        throw GatewayException("Invalid resource entry");
      }

      std::filesystem::path library_path = parts[1];
      if (!library_path.is_absolute()) {
        library_path = (result.resourcePath.value() / library_path);
      }
      resources.push_back({parts[0], library_path.string()});
    }
    return resources;
  }

  std::shared_ptr<rclcpp_components::NodeFactory>
  create_component_factory(const std::string & package_name, const std::string & class_name)
  {
    auto resources = get_component_resources(package_name, "rclcpp_components");
    
    std::string library_path = resources[0].second;
    std::string fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";

    class_loader::ClassLoader * loader;
    RCLCPP_DEBUG(get_logger(), "Load Library: %s", library_path.c_str());
    try {
      loader_ = std::make_unique<class_loader::ClassLoader>(library_path);
    } catch (const std::exception & ex) {
      throw GatewayException("Failed to load library: " + std::string(ex.what()));
    } catch (...) {
      throw GatewayException("Failed to load library");
    }
  
    loader = loader_.get();

    auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (const auto & clazz : classes) {
      RCLCPP_DEBUG(get_logger(), "Found class: %s", clazz.c_str());
      if (clazz == class_name || clazz == fq_class_name) {
        RCLCPP_DEBUG(get_logger(), "Instantiate class: %s", clazz.c_str());
        return loader->createInstance<rclcpp_components::NodeFactory>(clazz);
      }
    }
    return {};
  }

  rclcpp::Service<crazyflie_interfaces::srv::AddCrazyflie>::SharedPtr add_service_;
  rclcpp::Service<crazyflie_interfaces::srv::RemoveCrazyflie>::SharedPtr remove_service_;
  rclcpp::TimerBase::SharedPtr cleanup_timer_;

  std::unique_ptr<class_loader::ClassLoader> loader_;
  std::shared_ptr<rclcpp_components::NodeFactory> factory_;
  std::map<std::pair<uint8_t, uint8_t>, std::pair<rclcpp_components::NodeInstanceWrapper, DedicatedExecutorWrapper>> crazyflies_; // id, channel

protected:
  std::weak_ptr<rclcpp::Executor> executor_;
};

void sigint_handler(int signum)
{
    (void)signum;
    sigint_received.store(true);
    int safey_counter = 0;
    while (!gateway_shutdown_done.load())
    { 
        safey_counter++;
        if (safey_counter > 500) break;// 3 seconds timeout
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (gateway_shutdown_done.load()) std::cerr << "Gateway shut down cleanly after SIGINT." << std::endl;
    else std::cerr << "Gateway shutdown after SIGINT timed out." << std::endl;
}


int main(int argc, char ** argv)
{
  signal(SIGINT, sigint_handler);
  // Install before rclcpp this way rclcpp will store it as a "old" handler and execute it before its own shutdown


  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<CrazyflieGateway>(exec, options);
  exec->add_node(node);
  exec->spin();
  exec->remove_node(node);
  rclcpp::shutdown();
  return 0;
}
