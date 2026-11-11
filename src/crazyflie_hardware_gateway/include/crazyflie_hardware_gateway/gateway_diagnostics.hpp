#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crazyflie_hardware_gateway {

struct DiagnosticsNodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
};

// Capture under the gateway's entry lock, then publish after releasing it.
// Ready means configured, not flight readiness. Removing includes shutdown/cleanup.
// Pending counts include received service requests only, not middleware queues.
struct GatewaySnapshot
{
  size_t configuring{0};
  size_t ready{0};
  size_t removing{0};
  size_t pending_adds{0};
  size_t pending_removes{0};
  double oldest_pending_add_seconds{0.0};
  double oldest_pending_remove_seconds{0.0};
  bool shutting_down{false};
};

// Passive helper: creates no timers and does not access gateway entries.
// Record methods and publish() are safe to call from different callback groups.
class GatewayDiagnostics
{
public:
  explicit GatewayDiagnostics(DiagnosticsNodeInterfaces interfaces);
  void record_add_result(bool success);
  void record_remove_result(bool success);
  void record_response_failure();
  // Called once per completed processing tick, using steady-clock times.
  void record_processing(std::chrono::steady_clock::time_point started,
    std::chrono::steady_clock::time_point finished);
  void publish(const GatewaySnapshot & snapshot);
  // Interval-scoped: a subsequent healthy reporting interval clears the error.
  void report_error(const std::string & message);

private:
  DiagnosticsNodeInterfaces m_interfaces;
  double m_pending_warn_seconds;
  double m_processing_warn_ms;
  std::mutex m_mutex;
  uint64_t m_adds_succeeded{0};
  uint64_t m_adds_failed{0};
  uint64_t m_removes_completed{0};
  uint64_t m_removes_rejected{0};
  uint64_t m_response_failures{0};
  bool m_interval_failure{false};
  std::string m_interval_error;
  std::chrono::steady_clock::time_point m_last_processing{};
  double m_max_processing_interval_ms{0.0};
  double m_max_processing_duration_ms{0.0};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_publisher;
};

}  // namespace crazyflie_hardware_gateway
