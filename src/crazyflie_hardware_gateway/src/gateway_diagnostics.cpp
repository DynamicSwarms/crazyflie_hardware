#include "crazyflie_hardware_gateway/gateway_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace crazyflie_hardware_gateway {
namespace {
double declare_threshold(const DiagnosticsNodeInterfaces & interfaces,
  const std::string & name, double default_value, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  // The gateway automatically declares parameters supplied through overrides.
  const double value = interfaces.parameters->has_parameter(name) ?
    interfaces.parameters->get_parameter(name).as_double() :
    interfaces.parameters->declare_parameter(
    name, rclcpp::ParameterValue(default_value), descriptor).get<double>();
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(name + " must be finite and nonnegative");
  }
  return value;
}
}  // namespace

GatewayDiagnostics::GatewayDiagnostics(DiagnosticsNodeInterfaces interfaces)
: m_interfaces(std::move(interfaces)),
  m_pending_warn_seconds(declare_threshold(m_interfaces,
    "diagnostics_pending_warn_seconds", 15.0,
    "Pending request age warning threshold in seconds; 0 disables")),
  m_processing_warn_ms(declare_threshold(m_interfaces,
    "diagnostics_processing_warn_ms", 500.0,
    "Processing interval/duration warning threshold in milliseconds; 0 disables"))
{
  m_publisher = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    m_interfaces.parameters, m_interfaces.topics, "/diagnostics", rclcpp::QoS(10));
}

void GatewayDiagnostics::record_add_result(bool success)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (success) ++m_adds_succeeded;
  else {++m_adds_failed; m_interval_failure = true;}
}

void GatewayDiagnostics::record_remove_result(bool success)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (success) ++m_removes_completed;
  else {++m_removes_rejected; m_interval_failure = true;}
}

void GatewayDiagnostics::record_response_failure()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  ++m_response_failures;
  m_interval_failure = true;
}

void GatewayDiagnostics::record_processing(std::chrono::steady_clock::time_point started,
  std::chrono::steady_clock::time_point finished)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_last_processing != std::chrono::steady_clock::time_point{}) {
    m_max_processing_interval_ms = std::max(m_max_processing_interval_ms,
      std::chrono::duration<double, std::milli>(started - m_last_processing).count());
  }
  m_last_processing = started;
  m_max_processing_duration_ms = std::max(m_max_processing_duration_ms,
    std::chrono::duration<double, std::milli>(finished - started).count());
}

void GatewayDiagnostics::report_error(const std::string & message)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_interval_error = message;
}

void GatewayDiagnostics::publish(const GatewaySnapshot & snapshot)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  Status status;
  status.name = std::string(m_interfaces.base->get_fully_qualified_name()) + "/gateway";
  status.hardware_id = "gateway";
  status.level = Status::OK;
  status.message = snapshot.shutting_down ? "Shutting down" : "Running";
  const auto add = [&status](const std::string & key, const auto & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      status.values.push_back(std::move(item));
    };
  add("Managed entries", snapshot.configuring + snapshot.ready + snapshot.removing);
  add("Configuring", snapshot.configuring);
  add("Ready (configured)", snapshot.ready);
  add("Removing (including cleanup)", snapshot.removing);
  add("Pending add requests", snapshot.pending_adds);
  add("Pending remove requests", snapshot.pending_removes);
  add("Oldest pending add age (s)", snapshot.oldest_pending_add_seconds);
  add("Oldest pending remove age (s)", snapshot.oldest_pending_remove_seconds);
  add("Successful adds total", m_adds_succeeded);
  add("Failed adds total", m_adds_failed);
  add("Completed removes total", m_removes_completed);
  add("Rejected removes total", m_removes_rejected);
  add("Response delivery failures total", m_response_failures);
  const double processing_age_ms = m_last_processing == std::chrono::steady_clock::time_point{} ?
    0.0 : std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - m_last_processing).count();
  add("Processing observed", m_last_processing != std::chrono::steady_clock::time_point{});
  add("Last processing tick age (ms)", processing_age_ms);
  add("Maximum processing interval in interval (ms)", m_max_processing_interval_ms);
  add("Maximum processing duration in interval (ms)", m_max_processing_duration_ms);
  add("Shutting down", snapshot.shutting_down);
  add("Pending age warning threshold (s; 0 disables)", m_pending_warn_seconds);
  add("Processing warning threshold (ms; 0 disables)", m_processing_warn_ms);

  const bool pending_delayed = m_pending_warn_seconds > 0.0 &&
    ((snapshot.pending_adds > 0 && snapshot.oldest_pending_add_seconds > m_pending_warn_seconds) ||
    (snapshot.pending_removes > 0 && snapshot.oldest_pending_remove_seconds > m_pending_warn_seconds));
  const bool processing_delayed = m_processing_warn_ms > 0.0 &&
    std::max({processing_age_ms, m_max_processing_interval_ms,
      m_max_processing_duration_ms}) > m_processing_warn_ms;
  if (!m_interval_error.empty()) {
    status.level = Status::ERROR;
    status.message = m_interval_error;
  } else if (pending_delayed || processing_delayed || m_interval_failure) {
    status.level = Status::WARN;
    status.message.clear();
    if (pending_delayed) status.message += "Pending request delayed; ";
    if (processing_delayed) status.message += "Gateway processing delayed; ";
    if (m_interval_failure) status.message += "Request or response failure in interval; ";
  } else if (!snapshot.shutting_down &&
    snapshot.configuring + snapshot.ready + snapshot.removing == 0) {
    status.message = "Running; no managed entries";
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = m_interfaces.clock->get_clock()->now();
  diagnostics.status.push_back(std::move(status));
  m_publisher->publish(diagnostics);
  m_interval_failure = false;
  m_interval_error.clear();
  m_max_processing_interval_ms = 0.0;
  m_max_processing_duration_ms = 0.0;
}

}  // namespace crazyflie_hardware_gateway
