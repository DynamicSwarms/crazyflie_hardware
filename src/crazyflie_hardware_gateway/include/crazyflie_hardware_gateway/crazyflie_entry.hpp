#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "crazyflie_hardware_gateway/gateway_diagnostics.hpp"
#include "rclcpp_components/node_instance_wrapper.hpp"
#include "crazyflie_hardware_gateway/crazyflie_lifecycle_client.hpp"
#include "crazyflie_hardware_gateway/dedicated_executor_wrapper.hpp"

class CrazyflieEntry
{
public:
  using Respond = std::function<void(bool, const std::string &)>;

  CrazyflieEntry(int id, Respond respond_to_add, rclcpp::Logger logger);
  void initialize(rclcpp_components::NodeInstanceWrapper node,
    const GatewayNodeInterfaces & gateway,
    rclcpp::CallbackGroup::SharedPtr callback_group);
  void initialization_failed(const std::string & reason);
  void request_removal(Respond respond = {});

  // Only process_crazyflies calls poll(). Returns true after resources are released
  // so the gateway may erase the entry before completing pending requests.
  bool poll();
  void complete_removal();
  void append_diagnostics(crazyflie_hardware_gateway::GatewaySnapshot & snapshot,
    std::chrono::steady_clock::time_point now) const;

private:
  enum class Phase { Configuring, Ready, Removing, ShuttingDown, Stopping, Removed };
  void advance();
  void begin_stop();

  // Entry methods are serialized by the gateway's m_entries_mutex.
  bool m_initialized{false};
  bool m_remove_requested{false};
  Respond m_respond_to_add;
  std::vector<Respond> m_respond_to_remove;
  bool m_removed = false;
  const std::chrono::steady_clock::time_point m_add_started{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point m_remove_started{};

  rclcpp::Logger m_logger;
  Phase m_phase = Phase::Configuring;
  bool m_configure_started = false;
  std::string m_failure;
  std::chrono::steady_clock::time_point m_discovery_deadline;
  std::optional<rclcpp_components::NodeInstanceWrapper> m_node;
  std::unique_ptr<CrazyflieLifecycleClient> m_lifecycle;
  DedicatedExecutorWrapper m_execution;
};
