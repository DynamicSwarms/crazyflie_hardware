#include <cstdio>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_components/node_factory.hpp"
#include "crazyflie_hardware_gateway/crazyflie_entry.hpp"

#include "ament_index_cpp/get_resource.hpp"
#include "rcpputils/split.hpp"
#include "class_loader/class_loader.hpp"


#include "crazyflie_interfaces/srv/add_crazyflie.hpp"
#include "crazyflie_interfaces/srv/remove_crazyflie.hpp"

#include <memory>
#include <filesystem>
#include <mutex>
#include <utility>
#include <map>
#include <iomanip>
#include <sstream>

#include "signal.h"
#include <pthread.h>


class GatewayException : public std::runtime_error
{
public:
  explicit GatewayException(const std::string & error_desc)
  : std::runtime_error(error_desc) {}
};

using Add = crazyflie_interfaces::srv::AddCrazyflie;
using Remove = crazyflie_interfaces::srv::RemoveCrazyflie;
using Key = std::pair<uint8_t, uint8_t>;

class CrazyflieGateway : public rclcpp::Node
{
public:
  CrazyflieGateway(
    std::weak_ptr<rclcpp::Executor> executor,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("crazyflie_hardware_gateway", options)
  , m_crazyflies()
  , m_executor(executor)
  {
    m_diagnostics = std::make_unique<crazyflie_hardware_gateway::GatewayDiagnostics>(
      crazyflie_hardware_gateway::DiagnosticsNodeInterfaces{
        get_node_base_interface(), get_node_clock_interface(),
        get_node_parameters_interface(), get_node_topics_interface()});

    auto service_qos = rclcpp::ServicesQoS().keep_last(100); // This way it is possible to queue up multiple add requestst

    m_requests_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    m_processing_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_add_service = this->create_service<crazyflie_interfaces::srv::AddCrazyflie>(
      "~/add_crazyflie", 
      std::bind(&CrazyflieGateway::handle_add_crazyflie, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      service_qos,
      m_requests_group);

    m_remove_service = this->create_service<crazyflie_interfaces::srv::RemoveCrazyflie>(
      "~/remove_crazyflie",
      std::bind(&CrazyflieGateway::handle_remove_crazyflie, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      service_qos,
      m_requests_group);

    m_processing_timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CrazyflieGateway::process_crazyflies, this),
      m_processing_group);

    m_diagnostics_timer = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CrazyflieGateway::publish_diagnostics, this));

    m_factory = create_component_factory("crazyflie_hardware", "CrazyflieNode");

    if (!m_factory) throw GatewayException("Crazyflie component factory not found.");
    RCLCPP_INFO(get_logger(), "Crazyflie Gateway ready.");
  }

  void shutdown_all_crazyflies()
  {
    std::lock_guard<std::mutex> lock(m_entries_mutex);

    if (m_shutting_down) return;
    m_shutting_down = true;
    if (!m_crazyflies.empty()) RCLCPP_INFO(get_logger(), "Shutting down all crazyflies due to SIGINT. This might take a moment.");
    
    for (auto & [key, entry] : m_crazyflies) {
      entry->request_removal();
    }
  }

  size_t remaining_crazyflies()
  {
    std::lock_guard<std::mutex> lock(m_entries_mutex);
    return m_crazyflies.size();
  }

private:
  void 
  add_crazyflie(
    int id,
    int channel,
    const geometry_msgs::msg::Pose & initial_pose,
    const std::string & type,
    CrazyflieEntry::Respond respond)
  {
    std::lock_guard<std::mutex> lock(m_entries_mutex);
    if (m_shutting_down) {
      respond(false, "Gateway is shutting down; cannot add Crazyflie.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Adding Crazyflie with Channel: %d, Id: %d", channel, id);
    Key crazyflie_key = {id, channel};
    auto entry = std::make_shared<CrazyflieEntry>(id, respond, get_logger());
    
    // IDs must be unique across channels because they determine the ROS namespace.
    const bool id_conflict = std::any_of(
      m_crazyflies.begin(), m_crazyflies.end(),
      [id](const auto & entry) { return entry.first.first == id; });
    
    if (id_conflict || !m_crazyflies.emplace(crazyflie_key, entry).second) {
      respond(false, "Crazyflie already exists!");
      return;
    }

    try {
      auto options = create_node_options(id, channel, initial_pose, type);
      const GatewayNodeInterfaces interfaces{
        get_node_base_interface(), get_node_graph_interface(), get_node_services_interface(),
        get_node_topics_interface()};
      auto node = m_factory->create_node_instance(options);
      entry->initialize(std::move(node), interfaces, m_requests_group);
    } catch (const std::exception & ex) {
      entry->initialization_failed(ex.what());
    }
  }

  void 
  remove_crazyflie(
    int id, 
    int channel, 
    CrazyflieEntry::Respond respond)
  {
    std::lock_guard<std::mutex> lock(m_entries_mutex);
    RCLCPP_INFO(get_logger(), "Removing Crazyflie with Channel: %d, Id: %d", channel, id);
    Key crazyflie_key = {id, channel};
    const auto cf = m_crazyflies.find(crazyflie_key);
    if (cf == m_crazyflies.end()) {
      respond(false,
        "Couldn't remove Crazyflie with Channel: " + std::to_string(channel)
        + ", ID: " + std::to_string(id) + "; not in list.");
      return;
    }
    cf->second->request_removal(std::move(respond));
  }

private:
  void process_crazyflies()
  {
    const auto started = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(m_entries_mutex);
    for (auto it = m_crazyflies.begin(); it != m_crazyflies.end();) {
      if (it->second->poll()) {
        auto entry = it->second;
        it = m_crazyflies.erase(it);
        entry->complete_removal();
      } else {
        ++it;
      }
    }
    lock.unlock();
    m_diagnostics->record_processing(started, std::chrono::steady_clock::now());
  }

  void publish_diagnostics()
  {
    crazyflie_hardware_gateway::GatewaySnapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(m_entries_mutex);
      const auto now = std::chrono::steady_clock::now();
      snapshot.shutting_down = m_shutting_down;
      for (const auto & [key, entry] : m_crazyflies) {
        entry->append_diagnostics(snapshot, now);
      }
    }
    m_diagnostics->publish(snapshot);
  }

  void handle_add_crazyflie(
    const std::shared_ptr<rclcpp::Service<Add>> service_handle,
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<Add::Request> request)
  {
    try {
      auto [id, channel] = uri_to_id_channel(request->uri);
      add_crazyflie(id, channel, request->initial_pose, request->type,
        std::bind(&CrazyflieGateway::respond_add, this, service_handle, header,
          std::placeholders::_1, std::placeholders::_2));
    } catch (const std::exception & ex) {
      respond_add(service_handle, header, false, ex.what());
    }
  }

  void handle_remove_crazyflie(
    const std::shared_ptr<rclcpp::Service<Remove>> service_handle,
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<Remove::Request> request)
  {
    try {
      auto [id, channel] = uri_to_id_channel(request->uri);
      remove_crazyflie(
        id, 
        channel,
        std::bind(&CrazyflieGateway::respond_remove, this, service_handle, header,
          std::placeholders::_1, std::placeholders::_2));
    } catch (const std::exception & ex) {
      respond_remove(service_handle, header, false, ex.what());
    }
  }

  void respond_add(
    const std::shared_ptr<rclcpp::Service<Add>> & service,
    const std::shared_ptr<rmw_request_id_t> & header, 
    bool success, 
    const std::string & message)
  {
    m_diagnostics->record_add_result(success);
    Add::Response response;
    response.success = success;
    response.msg = message;
    try { 
      service->send_response(*header, response); 
    } catch (const std::exception & ex) {
      m_diagnostics->record_response_failure();
      RCLCPP_WARN(get_logger(), "Could not send add response: %s", ex.what());
    }
  }

  void respond_remove(
    const std::shared_ptr<rclcpp::Service<Remove>> & service,
    const std::shared_ptr<rmw_request_id_t> & header, 
    bool success, 
    const std::string & message)
  {
    m_diagnostics->record_remove_result(success);
    Remove::Response response;
    response.success = success;
    response.msg = message;
    try { 
      service->send_response(*header, response); 
    } catch (const std::exception & ex) {
      m_diagnostics->record_response_failure();
      RCLCPP_WARN(get_logger(), "Could not send remove response: %s", ex.what());
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
        if (channel < 0 || channel > 125) {
          throw GatewayException("Channel out of range [0,125]");
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
    pos_stream << std::fixed << std::setprecision(3) << "[" 
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
      m_loader = std::make_unique<class_loader::ClassLoader>(library_path);
    } catch (const std::exception & ex) {
      throw GatewayException("Failed to load library: " + std::string(ex.what()));
    } catch (...) {
      throw GatewayException("Failed to load library");
    }
  
    loader = m_loader.get();

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

  rclcpp::Service<crazyflie_interfaces::srv::AddCrazyflie>::SharedPtr m_add_service;
  rclcpp::Service<crazyflie_interfaces::srv::RemoveCrazyflie>::SharedPtr m_remove_service;
  rclcpp::TimerBase::SharedPtr m_processing_timer;
  std::unique_ptr<crazyflie_hardware_gateway::GatewayDiagnostics> m_diagnostics;
  rclcpp::TimerBase::SharedPtr m_diagnostics_timer;

  std::unique_ptr<class_loader::ClassLoader> m_loader;
  std::shared_ptr<rclcpp_components::NodeFactory> m_factory;
  rclcpp::CallbackGroup::SharedPtr m_requests_group;
  rclcpp::CallbackGroup::SharedPtr m_processing_group;
  std::mutex m_entries_mutex;
  bool m_shutting_down{false};  // Protected by m_entries_mutex.
  std::map<Key, std::shared_ptr<CrazyflieEntry>> m_crazyflies;

protected:
  std::weak_ptr<rclcpp::Executor> m_executor;
};

void signal_thread(
    std::shared_ptr<CrazyflieGateway> gateway,
    rclcpp::executors::MultiThreadedExecutor * executor)
{
  sigset_t set;
  sigemptyset(&set);
  sigaddset(&set, SIGINT);

  int sig;
  sigwait(&set, &sig);

  gateway->shutdown_all_crazyflies();
  int safety_counter = 0;
  size_t last_remaining = gateway->remaining_crazyflies();
  auto start_time = std::chrono::steady_clock::now();
  while (gateway->remaining_crazyflies() != 0)
  {
    size_t remaining = gateway->remaining_crazyflies();
    if (remaining != last_remaining) safety_counter = 0; // reset counter if there is progress
    last_remaining = remaining;

    if (start_time + std::chrono::milliseconds(500) < std::chrono::steady_clock::now()) {
      RCLCPP_INFO(gateway->get_logger(),
        "Waiting for gateway to shut down cleanly after SIGINT. Remaining crazyflies: %zu", remaining);
      start_time = std::chrono::steady_clock::now();
    }

    safety_counter++;
    if (safety_counter > 300) break; // 3 seconds without progress
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (gateway->remaining_crazyflies() != 0) {
    RCLCPP_ERROR(gateway->get_logger(), "Gateway shutdown after SIGINT timed out.");
  }

  else {
    RCLCPP_INFO(gateway->get_logger(), "Gateway shut down cleanly after SIGINT.");
  }

  executor->cancel();
}

void block_sigint()
{
  sigset_t set;
  sigemptyset(&set);
  sigaddset(&set, SIGINT);
  pthread_sigmask(SIG_BLOCK, &set, nullptr);
}

int main(int argc, char ** argv)
{
  block_sigint();

  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<CrazyflieGateway>(exec, options);
  std::thread sig_thread(signal_thread, node, exec.get());

  exec->add_node(node);
  exec->spin();

  sig_thread.join();

  exec->remove_node(node);
  node.reset();
  exec.reset();
  rclcpp::shutdown();
  return 0;
}
