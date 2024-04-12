// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crtp_interface:msg/CrtpPacket.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__STRUCT_HPP_
#define CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__crtp_interface__msg__CrtpPacket __attribute__((deprecated))
#else
# define DEPRECATED__crtp_interface__msg__CrtpPacket __declspec(deprecated)
#endif

namespace crtp_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CrtpPacket_
{
  using Type = CrtpPacket_<ContainerAllocator>;

  explicit CrtpPacket_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->port = 0;
      this->channel = 0;
      std::fill<typename std::array<uint8_t, 31>::iterator, uint8_t>(this->data.begin(), this->data.end(), 0);
      this->data_length = 0;
    }
  }

  explicit CrtpPacket_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->port = 0;
      this->channel = 0;
      std::fill<typename std::array<uint8_t, 31>::iterator, uint8_t>(this->data.begin(), this->data.end(), 0);
      this->data_length = 0;
    }
  }

  // field types and members
  using _port_type =
    uint8_t;
  _port_type port;
  using _channel_type =
    uint8_t;
  _channel_type channel;
  using _data_type =
    std::array<uint8_t, 31>;
  _data_type data;
  using _data_length_type =
    uint8_t;
  _data_length_type data_length;

  // setters for named parameter idiom
  Type & set__port(
    const uint8_t & _arg)
  {
    this->port = _arg;
    return *this;
  }
  Type & set__channel(
    const uint8_t & _arg)
  {
    this->channel = _arg;
    return *this;
  }
  Type & set__data(
    const std::array<uint8_t, 31> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__data_length(
    const uint8_t & _arg)
  {
    this->data_length = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crtp_interface::msg::CrtpPacket_<ContainerAllocator> *;
  using ConstRawPtr =
    const crtp_interface::msg::CrtpPacket_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crtp_interface::msg::CrtpPacket_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crtp_interface::msg::CrtpPacket_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crtp_interface::msg::CrtpPacket_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::msg::CrtpPacket_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crtp_interface::msg::CrtpPacket_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::msg::CrtpPacket_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crtp_interface::msg::CrtpPacket_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crtp_interface::msg::CrtpPacket_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crtp_interface__msg__CrtpPacket
    std::shared_ptr<crtp_interface::msg::CrtpPacket_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crtp_interface__msg__CrtpPacket
    std::shared_ptr<crtp_interface::msg::CrtpPacket_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CrtpPacket_ & other) const
  {
    if (this->port != other.port) {
      return false;
    }
    if (this->channel != other.channel) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->data_length != other.data_length) {
      return false;
    }
    return true;
  }
  bool operator!=(const CrtpPacket_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CrtpPacket_

// alias to use template instance with default allocator
using CrtpPacket =
  crtp_interface::msg::CrtpPacket_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace crtp_interface

#endif  // CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__STRUCT_HPP_
