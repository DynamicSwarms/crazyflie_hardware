#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "rclcpp/executor.hpp"
#include "rclcpp/logger.hpp"

struct DedicatedExecutorWrapper
{
  explicit DedicatedExecutorWrapper(rclcpp::Logger logger);
  ~DedicatedExecutorWrapper();
  void start(std::shared_ptr<rclcpp::Executor> exec);
  void stop();
  bool finished() const;
  void join();

private:
  rclcpp::Logger m_logger;
  std::shared_ptr<rclcpp::Executor> m_executor;
  std::thread m_thread;
  std::atomic_bool m_thread_initialized{false};
  std::atomic_bool m_finished{true};
};

