// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crtp_interface:msg/SetAutoping.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__STRUCT_HPP_
#define CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__crtp_interface__msg__SetAutoping __attribute__((deprecated))
#else
# define DEPRECATED__crtp_interface__msg__SetAutoping __declspec(deprecated)
#endif

namespace crtp_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SetAutoping_
{
  using Type = SetAutoping_<ContainerAllocator>;

  explicit SetAutoping_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
      std::fill<typename std::array<uint8_t, 5>::iterator, uint8_t>(this->address.begin(), this->address.end(), 0);
      this->datarate = 0;
      this->rate = 0;
    }
  }

  explicit SetAutoping_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : address(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
      std::fill<typename std::array<uint8_t, 5>::iterator, uint8_t>(this->address.begin(), this->address.end(), 0);
      this->datarate = 0;
      this->rate = 0;
    }
  }

  // field types and members
  using _channel_type =
    uint8_t;
  _channel_type channel;
  using _address_type =
    std::array<uint8_t, 5>;
  _address_type address;
  using _datarate_type =
    uint8_t;
  _datarate_type datarate;
  using _rate_type =
    uint16_t;
  _rate_type rate;

  // setters for named parameter idiom
  Type & set__channel(
    const uint8_t & _arg)
  {
    this->channel = _arg;
    return *this;
  }
  Type & set__address(
    const std::array<uint8_t, 5> & _arg)
  {
    this->address = _arg;
    return *this;
  }
  Type & set__datarate(
    const uint8_t & _arg)
  {
    this->datarate = _arg;
    return *this;
  }
  Type & set__rate(
    const uint16_t & _arg)
  {
    this->rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crtp_interface::msg::SetAutoping_<ContainerAllocator> *;
  using ConstRawPtr =
    const crtp_interface::msg::SetAutoping_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crtp_interface::msg::SetAutoping_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crtp_interface::msg::SetAutoping_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crtp_interface::msg::SetAutoping_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::msg::SetAutoping_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crtp_interface::msg::SetAutoping_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::msg::SetAutoping_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crtp_interface::msg::SetAutoping_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crtp_interface::msg::SetAutoping_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crtp_interface__msg__SetAutoping
    std::shared_ptr<crtp_interface::msg::SetAutoping_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crtp_interface__msg__SetAutoping
    std::shared_ptr<crtp_interface::msg::SetAutoping_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetAutoping_ & other) const
  {
    if (this->channel != other.channel) {
      return false;
    }
    if (this->address != other.address) {
      return false;
    }
    if (this->datarate != other.datarate) {
      return false;
    }
    if (this->rate != other.rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetAutoping_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetAutoping_

// alias to use template instance with default allocator
using SetAutoping =
  crtp_interface::msg::SetAutoping_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace crtp_interface

#endif  // CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__STRUCT_HPP_
