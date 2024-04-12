// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crtp_interface:msg/CrtpResponse.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__STRUCT_HPP_
#define CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'packet'
#include "crtp_interface/msg/detail/crtp_packet__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__crtp_interface__msg__CrtpResponse __attribute__((deprecated))
#else
# define DEPRECATED__crtp_interface__msg__CrtpResponse __declspec(deprecated)
#endif

namespace crtp_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CrtpResponse_
{
  using Type = CrtpResponse_<ContainerAllocator>;

  explicit CrtpResponse_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : packet(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
    }
  }

  explicit CrtpResponse_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : packet(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
    }
  }

  // field types and members
  using _channel_type =
    uint8_t;
  _channel_type channel;
  using _address_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _address_type address;
  using _packet_type =
    crtp_interface::msg::CrtpPacket_<ContainerAllocator>;
  _packet_type packet;

  // setters for named parameter idiom
  Type & set__channel(
    const uint8_t & _arg)
  {
    this->channel = _arg;
    return *this;
  }
  Type & set__address(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->address = _arg;
    return *this;
  }
  Type & set__packet(
    const crtp_interface::msg::CrtpPacket_<ContainerAllocator> & _arg)
  {
    this->packet = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crtp_interface::msg::CrtpResponse_<ContainerAllocator> *;
  using ConstRawPtr =
    const crtp_interface::msg::CrtpResponse_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crtp_interface::msg::CrtpResponse_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crtp_interface::msg::CrtpResponse_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crtp_interface::msg::CrtpResponse_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::msg::CrtpResponse_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crtp_interface::msg::CrtpResponse_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::msg::CrtpResponse_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crtp_interface::msg::CrtpResponse_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crtp_interface::msg::CrtpResponse_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crtp_interface__msg__CrtpResponse
    std::shared_ptr<crtp_interface::msg::CrtpResponse_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crtp_interface__msg__CrtpResponse
    std::shared_ptr<crtp_interface::msg::CrtpResponse_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CrtpResponse_ & other) const
  {
    if (this->channel != other.channel) {
      return false;
    }
    if (this->address != other.address) {
      return false;
    }
    if (this->packet != other.packet) {
      return false;
    }
    return true;
  }
  bool operator!=(const CrtpResponse_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CrtpResponse_

// alias to use template instance with default allocator
using CrtpResponse =
  crtp_interface::msg::CrtpResponse_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace crtp_interface

#endif  // CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__STRUCT_HPP_
