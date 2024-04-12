// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crtp_interface:msg/CrtpPacket.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__BUILDER_HPP_
#define CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crtp_interface/msg/detail/crtp_packet__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crtp_interface
{

namespace msg
{

namespace builder
{

class Init_CrtpPacket_data_length
{
public:
  explicit Init_CrtpPacket_data_length(::crtp_interface::msg::CrtpPacket & msg)
  : msg_(msg)
  {}
  ::crtp_interface::msg::CrtpPacket data_length(::crtp_interface::msg::CrtpPacket::_data_length_type arg)
  {
    msg_.data_length = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crtp_interface::msg::CrtpPacket msg_;
};

class Init_CrtpPacket_data
{
public:
  explicit Init_CrtpPacket_data(::crtp_interface::msg::CrtpPacket & msg)
  : msg_(msg)
  {}
  Init_CrtpPacket_data_length data(::crtp_interface::msg::CrtpPacket::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_CrtpPacket_data_length(msg_);
  }

private:
  ::crtp_interface::msg::CrtpPacket msg_;
};

class Init_CrtpPacket_channel
{
public:
  explicit Init_CrtpPacket_channel(::crtp_interface::msg::CrtpPacket & msg)
  : msg_(msg)
  {}
  Init_CrtpPacket_data channel(::crtp_interface::msg::CrtpPacket::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_CrtpPacket_data(msg_);
  }

private:
  ::crtp_interface::msg::CrtpPacket msg_;
};

class Init_CrtpPacket_port
{
public:
  Init_CrtpPacket_port()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CrtpPacket_channel port(::crtp_interface::msg::CrtpPacket::_port_type arg)
  {
    msg_.port = std::move(arg);
    return Init_CrtpPacket_channel(msg_);
  }

private:
  ::crtp_interface::msg::CrtpPacket msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crtp_interface::msg::CrtpPacket>()
{
  return crtp_interface::msg::builder::Init_CrtpPacket_port();
}

}  // namespace crtp_interface

#endif  // CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__BUILDER_HPP_
