#include "crazyflie_hardware_gateway/dedicated_executor_wrapper.hpp"

#include <chrono>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

DedicatedExecutorWrapper::DedicatedExecutorWrapper(rclcpp::Logger logger)
: m_logger(logger.get_child("executor")) {}

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
        RCLCPP_ERROR(m_logger, "%s", ex.what());
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

