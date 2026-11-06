#pragma once

#include <future>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

// Nonblocking lifecycle calls. Used only by the gateway's process_crazyflies callback.
class CrazyflieLifecycleClient
{
public:
  CrazyflieLifecycleClient(rclcpp::Node & gateway, const std::string & node_name,
    rclcpp::CallbackGroup::SharedPtr callback_group);

  bool available() const;
  void configure();
  void shutdown(uint8_t current_state);
  std::optional<bool> transition_result();
  std::optional<uint8_t> current_state();

private:
  using ChangeState = lifecycle_msgs::srv::ChangeState;
  using GetState = lifecycle_msgs::srv::GetState;
  void transition(uint8_t id);

  rclcpp::Client<ChangeState>::SharedPtr m_change_state;
  rclcpp::Client<GetState>::SharedPtr m_get_state;
  std::shared_future<ChangeState::Response::SharedPtr> m_transition;
  std::shared_future<GetState::Response::SharedPtr> m_state;
};
