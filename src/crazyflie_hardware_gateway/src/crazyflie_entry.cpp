#include "crazyflie_hardware_gateway/crazyflie_entry.hpp"

#include <utility>
#include <algorithm>
#include "lifecycle_msgs/msg/state.hpp"

CrazyflieEntry::CrazyflieEntry(int id, Respond respond_to_add, rclcpp::Logger logger)
: m_respond_to_add(std::move(respond_to_add))
, m_logger(logger.get_child("cf" + std::to_string(id)))
, m_execution(m_logger) {}

void CrazyflieEntry::initialize(
  rclcpp_components::NodeInstanceWrapper node,
  const GatewayNodeInterfaces & gateway,
  rclcpp::CallbackGroup::SharedPtr callback_group)
{
  m_lifecycle = std::make_unique<CrazyflieLifecycleClient>(
    gateway,
    node.get_node_base_interface()->get_fully_qualified_name(), 
    callback_group, m_logger);
  
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node.get_node_base_interface());
  m_node.emplace(std::move(node));
  m_execution.start(executor);
  m_discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  m_initialized = true;
}

void CrazyflieEntry::initialization_failed(const std::string & reason)
{
  m_failure = reason;
  m_phase = Phase::Removing;
  m_initialized = true;
}

void CrazyflieEntry::request_removal(Respond respond)
{
  if (m_removed) {
    if (respond) respond(true, "Crazyflie removed.");
    return;
  }
  m_remove_requested = true;
  if (respond) {
    if (m_respond_to_remove.empty()) m_remove_started = std::chrono::steady_clock::now();
    m_respond_to_remove.push_back(std::move(respond));
  }
}

bool CrazyflieEntry::poll()
{
  if (!m_initialized) return false;
  try {
    if (m_phase != Phase::Stopping && m_phase != Phase::Removed && m_execution.finished()) {
      if (m_failure.empty()) m_failure = "Crazyflie executor stopped unexpectedly.";
      begin_stop();
    }
    advance();
  } catch (const std::exception & ex) {
    // Keep the object alive: an unanswered lifecycle call is not proof that the
    // node has stopped executing a transition. Retry cleanup on the next tick.
    RCLCPP_ERROR(m_logger, "Lifecycle operation failed: %s", ex.what());
    m_failure = ex.what();
    m_remove_requested = true;
  }
  return m_phase == Phase::Removed;
}

void CrazyflieEntry::advance()
{
  using State = lifecycle_msgs::msg::State;
  switch (m_phase) {
    case Phase::Configuring:
      if (!m_configure_started) {
        if (m_remove_requested) {
          m_phase = Phase::Removing;
        } else if (m_lifecycle->available()) {
          m_lifecycle->configure();
          m_configure_started = true;
        } else if (std::chrono::steady_clock::now() >= m_discovery_deadline) {
          m_failure = "Timed out discovering lifecycle services.";
          begin_stop(); // No configure request was sent.
        }
      } else if (auto result = m_lifecycle->transition_result()) {
        if (*result) {
          if (m_remove_requested) {
            m_phase = Phase::Removing;
          } else {
            m_phase = Phase::Ready;
            auto respond = std::move(m_respond_to_add);
            if (respond) respond(true, "Configured successfully.");
          }
        } else {
          m_failure = "Configuration failed.";
          m_phase = Phase::Removing;
        }
      }
      break;

    case Phase::Ready:
      if (m_remove_requested) {
        m_phase = Phase::Removing;
      } else if (auto state = m_lifecycle->current_state()) {
        if (*state == State::PRIMARY_STATE_FINALIZED) {
          m_failure = "Crazyflie shut down.";
          begin_stop();
        }
      }
      break;

    case Phase::Removing:
      if (!m_configure_started) {
        begin_stop();
      } else if (auto state = m_lifecycle->current_state()) {
        if (*state == State::PRIMARY_STATE_FINALIZED) begin_stop();
        else if (*state == State::PRIMARY_STATE_UNCONFIGURED ||
          *state == State::PRIMARY_STATE_INACTIVE || *state == State::PRIMARY_STATE_ACTIVE) {
          m_lifecycle->shutdown(*state);
          m_phase = Phase::ShuttingDown;
        }
      }
      break;

    case Phase::ShuttingDown:
      if (auto result = m_lifecycle->transition_result()) {
        if (*result) begin_stop();
        else m_phase = Phase::Removing; // Retry using the latest transition event.
      }
      break;

    case Phase::Stopping:
      if (m_execution.finished()) {
        m_execution.join();
        m_lifecycle.reset();
        m_node.reset();
        m_phase = Phase::Removed;
      }
      break;

    case Phase::Removed:
      break;
  }
}

void CrazyflieEntry::begin_stop()
{
  m_phase = Phase::Stopping;
  m_execution.stop();
}

void CrazyflieEntry::complete_removal()
{
  m_removed = true;
  auto add = std::move(m_respond_to_add);
  auto removes = std::move(m_respond_to_remove);
  m_phase = Phase::Removed;
  if (add) add(false, m_failure.empty() ? "Removed during initialization." : m_failure);
  for (auto & respond : removes) respond(true, "Crazyflie removed.");
}

void CrazyflieEntry::append_diagnostics(
  crazyflie_hardware_gateway::GatewaySnapshot & snapshot,
  std::chrono::steady_clock::time_point now) const
{
  if (m_remove_requested || (m_phase != Phase::Configuring && m_phase != Phase::Ready)) {
    ++snapshot.removing;
  } else if (m_phase == Phase::Ready) {
    ++snapshot.ready;
  } else {
    ++snapshot.configuring;
  }
  if (m_respond_to_add) {
    ++snapshot.pending_adds;
    snapshot.oldest_pending_add_seconds = std::max(snapshot.oldest_pending_add_seconds,
      std::chrono::duration<double>(now - m_add_started).count());
  }
  if (!m_respond_to_remove.empty()) {
    snapshot.pending_removes += m_respond_to_remove.size();
    snapshot.oldest_pending_remove_seconds = std::max(snapshot.oldest_pending_remove_seconds,
      std::chrono::duration<double>(now - m_remove_started).count());
  }
}
