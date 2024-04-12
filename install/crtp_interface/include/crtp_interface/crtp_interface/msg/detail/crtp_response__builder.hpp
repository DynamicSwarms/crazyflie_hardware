// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crtp_interface:msg/CrtpResponse.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__BUILDER_HPP_
#define CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crtp_interface/msg/detail/crtp_response__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crtp_interface
{

namespace msg
{

namespace builder
{

class Init_CrtpResponse_packet
{
public:
  explicit Init_CrtpResponse_packet(::crtp_interface::msg::CrtpResponse & msg)
  : msg_(msg)
  {}
  ::crtp_interface::msg::CrtpResponse packet(::crtp_interface::msg::CrtpResponse::_packet_type arg)
  {
    msg_.packet = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crtp_interface::msg::CrtpResponse msg_;
};

class Init_CrtpResponse_address
{
public:
  explicit Init_CrtpResponse_address(::crtp_interface::msg::CrtpResponse & msg)
  : msg_(msg)
  {}
  Init_CrtpResponse_packet address(::crtp_interface::msg::CrtpResponse::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_CrtpResponse_packet(msg_);
  }

private:
  ::crtp_interface::msg::CrtpResponse msg_;
};

class Init_CrtpResponse_channel
{
public:
  Init_CrtpResponse_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CrtpResponse_address channel(::crtp_interface::msg::CrtpResponse::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_CrtpResponse_address(msg_);
  }

private:
  ::crtp_interface::msg::CrtpResponse msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crtp_interface::msg::CrtpResponse>()
{
  return crtp_interface::msg::builder::Init_CrtpResponse_channel();
}

}  // namespace crtp_interface

#endif  // CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__BUILDER_HPP_
