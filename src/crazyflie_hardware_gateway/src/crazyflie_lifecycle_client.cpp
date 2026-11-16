#include "crazyflie_hardware_gateway/crazyflie_lifecycle_client.hpp"

#include <utility>
#include "rclcpp/create_client.hpp"
#include "rclcpp/create_subscription.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

CrazyflieLifecycleClient::CrazyflieLifecycleClient(
  const GatewayNodeInterfaces & gateway,
  const std::string & node_name,
  rclcpp::CallbackGroup::SharedPtr callback_group, 
  rclcpp::Logger logger)
: m_logger(logger.get_child("lifecycle"))
{
  m_change_state = rclcpp::create_client<ChangeState>(
    gateway.base, 
    gateway.graph, 
    gateway.services, 
    node_name + "/change_state",
    rclcpp::ServicesQoS(), 
    callback_group);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;
  auto topics = gateway.topics;
  m_transition_events = rclcpp::create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    topics, node_name + "/transition_event", rclcpp::QoS(10),
    [state = m_state](lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr event) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->value = event->goal_state.id;
    }, options);
}

bool CrazyflieLifecycleClient::available() const
{
  return m_change_state->service_is_ready();
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

std::optional<bool> 
CrazyflieLifecycleClient::transition_result()
{
  if (!m_transition.valid() ||
    m_transition.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return {};
  auto future = std::exchange(m_transition, {});
  try {
    const bool success = future.get()->success;
    if (!success) {
      RCLCPP_WARN(m_logger, "Lifecycle transition response reported success=false.");
    }
    return success;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(m_logger, "Lifecycle response failed: %s", ex.what());
    return false;
  }
}

std::optional<uint8_t> 
CrazyflieLifecycleClient::current_state() const
{
  std::lock_guard<std::mutex> lock(m_state->mutex);
  return m_state->value;
}
