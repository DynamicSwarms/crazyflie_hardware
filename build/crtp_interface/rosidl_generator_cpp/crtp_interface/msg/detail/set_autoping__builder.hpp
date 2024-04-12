// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crtp_interface:msg/SetAutoping.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__BUILDER_HPP_
#define CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crtp_interface/msg/detail/set_autoping__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crtp_interface
{

namespace msg
{

namespace builder
{

class Init_SetAutoping_rate
{
public:
  explicit Init_SetAutoping_rate(::crtp_interface::msg::SetAutoping & msg)
  : msg_(msg)
  {}
  ::crtp_interface::msg::SetAutoping rate(::crtp_interface::msg::SetAutoping::_rate_type arg)
  {
    msg_.rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crtp_interface::msg::SetAutoping msg_;
};

class Init_SetAutoping_datarate
{
public:
  explicit Init_SetAutoping_datarate(::crtp_interface::msg::SetAutoping & msg)
  : msg_(msg)
  {}
  Init_SetAutoping_rate datarate(::crtp_interface::msg::SetAutoping::_datarate_type arg)
  {
    msg_.datarate = std::move(arg);
    return Init_SetAutoping_rate(msg_);
  }

private:
  ::crtp_interface::msg::SetAutoping msg_;
};

class Init_SetAutoping_address
{
public:
  explicit Init_SetAutoping_address(::crtp_interface::msg::SetAutoping & msg)
  : msg_(msg)
  {}
  Init_SetAutoping_datarate address(::crtp_interface::msg::SetAutoping::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_SetAutoping_datarate(msg_);
  }

private:
  ::crtp_interface::msg::SetAutoping msg_;
};

class Init_SetAutoping_channel
{
public:
  Init_SetAutoping_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetAutoping_address channel(::crtp_interface::msg::SetAutoping::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_SetAutoping_address(msg_);
  }

private:
  ::crtp_interface::msg::SetAutoping msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crtp_interface::msg::SetAutoping>()
{
  return crtp_interface::msg::builder::Init_SetAutoping_channel();
}

}  // namespace crtp_interface

#endif  // CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__BUILDER_HPP_
