#pragma once

#include <future>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "crazyflie_hardware_gateway/node_interfaces.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

// Nonblocking lifecycle calls. Used only by the gateway's process_crazyflies callback.
class CrazyflieLifecycleClient
{
public:
  CrazyflieLifecycleClient(const GatewayNodeInterfaces & gateway, const std::string & node_name,
    rclcpp::CallbackGroup::SharedPtr callback_group, rclcpp::Logger logger);

  bool available() const;
  void configure();
  void shutdown(uint8_t current_state);
  std::optional<bool> transition_result();
  std::optional<uint8_t> current_state() const;

private:
  using ChangeState = lifecycle_msgs::srv::ChangeState;
  // Shared with the subscription callback, which may outlive this client.
  struct ObservedState
  {
    std::mutex mutex;
    std::optional<uint8_t> value;
  };
  void transition(uint8_t id);

  rclcpp::Logger m_logger;
  rclcpp::Client<ChangeState>::SharedPtr m_change_state;
  std::shared_ptr<ObservedState> m_state = std::make_shared<ObservedState>();
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr m_transition_events;
  std::shared_future<ChangeState::Response::SharedPtr> m_transition;
};
