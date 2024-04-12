// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crtp_interface:srv/CrtpPacketSend.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__BUILDER_HPP_
#define CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crtp_interface/srv/detail/crtp_packet_send__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crtp_interface
{

namespace srv
{

namespace builder
{

class Init_CrtpPacketSend_Request_packet
{
public:
  explicit Init_CrtpPacketSend_Request_packet(::crtp_interface::srv::CrtpPacketSend_Request & msg)
  : msg_(msg)
  {}
  ::crtp_interface::srv::CrtpPacketSend_Request packet(::crtp_interface::srv::CrtpPacketSend_Request::_packet_type arg)
  {
    msg_.packet = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crtp_interface::srv::CrtpPacketSend_Request msg_;
};

class Init_CrtpPacketSend_Request_datarate
{
public:
  explicit Init_CrtpPacketSend_Request_datarate(::crtp_interface::srv::CrtpPacketSend_Request & msg)
  : msg_(msg)
  {}
  Init_CrtpPacketSend_Request_packet datarate(::crtp_interface::srv::CrtpPacketSend_Request::_datarate_type arg)
  {
    msg_.datarate = std::move(arg);
    return Init_CrtpPacketSend_Request_packet(msg_);
  }

private:
  ::crtp_interface::srv::CrtpPacketSend_Request msg_;
};

class Init_CrtpPacketSend_Request_address
{
public:
  explicit Init_CrtpPacketSend_Request_address(::crtp_interface::srv::CrtpPacketSend_Request & msg)
  : msg_(msg)
  {}
  Init_CrtpPacketSend_Request_datarate address(::crtp_interface::srv::CrtpPacketSend_Request::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_CrtpPacketSend_Request_datarate(msg_);
  }

private:
  ::crtp_interface::srv::CrtpPacketSend_Request msg_;
};

class Init_CrtpPacketSend_Request_channel
{
public:
  Init_CrtpPacketSend_Request_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CrtpPacketSend_Request_address channel(::crtp_interface::srv::CrtpPacketSend_Request::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_CrtpPacketSend_Request_address(msg_);
  }

private:
  ::crtp_interface::srv::CrtpPacketSend_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crtp_interface::srv::CrtpPacketSend_Request>()
{
  return crtp_interface::srv::builder::Init_CrtpPacketSend_Request_channel();
}

}  // namespace crtp_interface


namespace crtp_interface
{

namespace srv
{

namespace builder
{

class Init_CrtpPacketSend_Response_success
{
public:
  Init_CrtpPacketSend_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::crtp_interface::srv::CrtpPacketSend_Response success(::crtp_interface::srv::CrtpPacketSend_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crtp_interface::srv::CrtpPacketSend_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crtp_interface::srv::CrtpPacketSend_Response>()
{
  return crtp_interface::srv::builder::Init_CrtpPacketSend_Response_success();
}

}  // namespace crtp_interface

#endif  // CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__BUILDER_HPP_
