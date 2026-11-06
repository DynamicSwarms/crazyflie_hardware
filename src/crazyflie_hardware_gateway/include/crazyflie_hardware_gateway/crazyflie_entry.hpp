#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "rclcpp_components/node_factory.hpp"
#include "crazyflie_hardware_gateway/crazyflie_lifecycle_client.hpp"

struct DedicatedExecutorWrapper
{
  ~DedicatedExecutorWrapper();
  void start(std::shared_ptr<rclcpp::Executor> exec);
  void stop();
  bool finished() const;
  void join();

private:
  std::shared_ptr<rclcpp::Executor> m_executor;
  std::thread m_thread;
  std::atomic_bool m_thread_initialized{false};
  std::atomic_bool m_finished{true};
};

class CrazyflieEntry
{
public:
  using Respond = std::function<void(bool, const std::string &)>;

  explicit CrazyflieEntry(Respond respond_to_add);
  void initialize(rclcpp_components::NodeFactory & factory,
    const rclcpp::NodeOptions & options, rclcpp::Node & gateway,
    rclcpp::CallbackGroup::SharedPtr callback_group);
  void initialization_failed(const std::string & reason);
  void request_removal(Respond respond = {});

  // Only process_crazyflies calls poll(). Returns true after resources are released
  // so the gateway may erase the entry before completing pending requests.
  bool poll();
  void complete_removal();

private:
  enum class Phase { Configuring, Ready, Removing, ShuttingDown, Stopping, Removed };
  void advance();
  void begin_stop();

  // Add publishes initialization with release/acquire. Thereafter only poll()
  // accesses lifecycle/executor state; service callbacks only record intent.
  std::atomic_bool m_initialized{false};
  std::atomic_bool m_remove_requested{false};
  std::mutex m_requests_mutex;
  Respond m_respond_to_add;
  std::vector<Respond> m_respond_to_remove;
  bool m_removed = false;

  Phase m_phase = Phase::Configuring;
  bool m_configure_started = false;
  std::string m_failure;
  std::chrono::steady_clock::time_point m_discovery_deadline;
  std::optional<rclcpp_components::NodeInstanceWrapper> m_node;
  std::unique_ptr<CrazyflieLifecycleClient> m_lifecycle;
  DedicatedExecutorWrapper m_execution;
};
