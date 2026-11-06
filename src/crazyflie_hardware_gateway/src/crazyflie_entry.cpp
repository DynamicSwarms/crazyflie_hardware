#include "crazyflie_hardware_gateway/crazyflie_entry.hpp"

#include <utility>
#include "lifecycle_msgs/msg/state.hpp"

DedicatedExecutorWrapper::~DedicatedExecutorWrapper()
{
  stop();
  join();
}

void DedicatedExecutorWrapper::start(std::shared_ptr<rclcpp::Executor> exec)
{
  m_executor = std::move(exec);
  m_finished.store(false);
  try {
    m_thread = std::thread([this] {
      m_thread_initialized.store(true);
      try { m_executor->spin(); }
      catch (const std::exception & ex) {
        RCLCPP_ERROR(rclcpp::get_logger("CrazyflieExecutor"), "%s", ex.what());
      }
      m_finished.store(true);
    });
  } catch (...) {
    m_finished.store(true);
    throw;
  }
}

void DedicatedExecutorWrapper::stop()
{
  if (!m_thread.joinable() || m_finished.load()) return;
  // Existing startup workaround, retained for this change.
  if (!m_thread_initialized.load()) rclcpp::sleep_for(std::chrono::milliseconds(1));
  m_executor->cancel();
}

bool DedicatedExecutorWrapper::finished() const { return m_finished.load(); }
void DedicatedExecutorWrapper::join() { if (m_thread.joinable()) m_thread.join(); }

CrazyflieEntry::CrazyflieEntry(Respond respond_to_add)
: m_respond_to_add(std::move(respond_to_add)) {}

void CrazyflieEntry::initialize(rclcpp_components::NodeFactory & factory,
  const rclcpp::NodeOptions & options, rclcpp::Node & gateway,
  rclcpp::CallbackGroup::SharedPtr callback_group)
{
  auto node = factory.create_node_instance(options);
  m_lifecycle = std::make_unique<CrazyflieLifecycleClient>(gateway,
    node.get_node_base_interface()->get_fully_qualified_name(), callback_group);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node.get_node_base_interface());
  m_node.emplace(std::move(node));
  m_execution.start(executor);
  m_discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  m_initialized.store(true);
}

void CrazyflieEntry::initialization_failed(const std::string & reason)
{
  m_failure = reason;
  m_phase = Phase::Removing;
  m_initialized.store(true);
}

void CrazyflieEntry::request_removal(Respond respond)
{
  bool already_removed;
  {
    std::lock_guard<std::mutex> lock(m_requests_mutex);
    already_removed = m_removed;
    if (!already_removed) {
      m_remove_requested.store(true);
      if (respond) m_respond_to_remove.push_back(std::move(respond));
    }
  }
  if (already_removed && respond) respond(true, "Crazyflie removed.");
}

bool CrazyflieEntry::poll()
{
  if (!m_initialized.load()) return false;
  try {
    if (m_phase != Phase::Stopping && m_phase != Phase::Removed && m_execution.finished()) {
      if (m_failure.empty()) m_failure = "Crazyflie executor stopped unexpectedly.";
      begin_stop();
    }
    advance();
  } catch (const std::exception & ex) {
    // Keep the object alive: an unanswered lifecycle call is not proof that the
    // node has stopped executing a transition. Retry cleanup on the next tick.
    RCLCPP_ERROR(rclcpp::get_logger("CrazyflieEntry"), "Lifecycle operation failed: %s", ex.what());
    m_failure = ex.what();
    m_remove_requested.store(true);
  }
  return m_phase == Phase::Removed;
}

void CrazyflieEntry::advance()
{
  using State = lifecycle_msgs::msg::State;
  switch (m_phase) {
    case Phase::Configuring:
      if (!m_configure_started) {
        if (m_remove_requested.load()) {
          m_phase = Phase::Removing;
        } else if (m_lifecycle->available()) {
          m_lifecycle->configure();
          m_configure_started = true;
        } else if (std::chrono::steady_clock::now() >= m_discovery_deadline) {
          m_failure = "Timed out discovering lifecycle services.";
          begin_stop(); // No configure request was sent.
        }
      } else if (auto result = m_lifecycle->transition_result()) {
        if (*result) m_phase = Phase::Ready;
        else {
          m_failure = "Configuration failed.";
          m_phase = Phase::Removing;
        }
      }
      break;

    case Phase::Ready:
      if (m_remove_requested.load()) {
        m_phase = Phase::Removing;
      } else if (auto state = m_lifecycle->current_state()) {
        if (*state == State::PRIMARY_STATE_FINALIZED) {
          m_failure = "Crazyflie shut down.";
          begin_stop();
        } else {
          Respond respond;
          {
            std::lock_guard<std::mutex> lock(m_requests_mutex);
            if (!m_remove_requested.load()) respond = std::move(m_respond_to_add);
          }
          if (respond) respond(true, "Configured successfully.");
        }
      }
      break;

    case Phase::Removing:
      if (!m_configure_started) {
        begin_stop();
      } else if (auto state = m_lifecycle->current_state()) {
        if (*state == State::PRIMARY_STATE_FINALIZED) begin_stop();
        else {
          m_lifecycle->shutdown(*state);
          m_phase = Phase::ShuttingDown;
        }
      }
      break;

    case Phase::ShuttingDown:
      if (auto result = m_lifecycle->transition_result()) {
        if (*result) begin_stop();
        else m_phase = Phase::Removing; // Re-query; the node may have finalized itself.
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
  Respond add;
  std::vector<Respond> removes;
  {
    std::lock_guard<std::mutex> lock(m_requests_mutex);
    m_removed = true;
    add = std::move(m_respond_to_add);
    removes = std::move(m_respond_to_remove);
  }
  m_phase = Phase::Removed;
  if (add) add(false, m_failure.empty() ? "Removed during initialization." : m_failure);
  for (auto & respond : removes) respond(true, "Crazyflie removed.");
}
