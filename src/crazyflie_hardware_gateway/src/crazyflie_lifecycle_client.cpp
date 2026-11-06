#include "crazyflie_hardware_gateway/crazyflie_lifecycle_client.hpp"

#include <utility>
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

CrazyflieLifecycleClient::CrazyflieLifecycleClient(
  rclcpp::Node & gateway, const std::string & node_name,
  rclcpp::CallbackGroup::SharedPtr callback_group)
{
  m_change_state = gateway.create_client<ChangeState>(node_name + "/change_state",
    rclcpp::ServicesQoS(), callback_group);
  m_get_state = gateway.create_client<GetState>(node_name + "/get_state",
    rclcpp::ServicesQoS(), callback_group);
}

bool CrazyflieLifecycleClient::available() const
{
  return m_change_state->service_is_ready() && m_get_state->service_is_ready();
}

void CrazyflieLifecycleClient::transition(uint8_t id)
{
  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = id;
  m_transition = m_change_state->async_send_request(request).share();
}

void CrazyflieLifecycleClient::configure()
{
  transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

void CrazyflieLifecycleClient::shutdown(uint8_t current_state)
{
  using State = lifecycle_msgs::msg::State;
  using Transition = lifecycle_msgs::msg::Transition;
  switch (current_state) {
    case State::PRIMARY_STATE_UNCONFIGURED:
      transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
      break;
    case State::PRIMARY_STATE_INACTIVE:
      transition(Transition::TRANSITION_INACTIVE_SHUTDOWN);
      break;
    case State::PRIMARY_STATE_ACTIVE:
      transition(Transition::TRANSITION_ACTIVE_SHUTDOWN);
      break;
    default:
      throw std::runtime_error("Cannot shut down from a transitional lifecycle state");
  }
}

std::optional<bool> CrazyflieLifecycleClient::transition_result()
{
  if (!m_transition.valid() ||
    m_transition.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return {};
  auto future = std::exchange(m_transition, {});
  try {
    return future.get()->success;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("CrazyflieLifecycleClient"),
      "Lifecycle response failed: %s", ex.what());
    return false;
  }
}

std::optional<uint8_t> CrazyflieLifecycleClient::current_state()
{
  if (!m_state.valid()) {
    if (m_get_state->service_is_ready()) {
      m_state = m_get_state->async_send_request(std::make_shared<GetState::Request>()).share();
    }
    return {};
  }
  if (m_state.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return {};
  auto future = std::exchange(m_state, {});
  return future.get()->current_state.id;
}
