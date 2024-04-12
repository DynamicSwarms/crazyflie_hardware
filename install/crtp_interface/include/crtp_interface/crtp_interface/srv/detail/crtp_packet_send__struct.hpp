// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crtp_interface:srv/CrtpPacketSend.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__STRUCT_HPP_
#define CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__STRUCT_HPP_

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
# define DEPRECATED__crtp_interface__srv__CrtpPacketSend_Request __attribute__((deprecated))
#else
# define DEPRECATED__crtp_interface__srv__CrtpPacketSend_Request __declspec(deprecated)
#endif

namespace crtp_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CrtpPacketSend_Request_
{
  using Type = CrtpPacketSend_Request_<ContainerAllocator>;

  explicit CrtpPacketSend_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : packet(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
      std::fill<typename std::array<uint8_t, 5>::iterator, uint8_t>(this->address.begin(), this->address.end(), 0);
      this->datarate = 0;
    }
  }

  explicit CrtpPacketSend_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : address(_alloc),
    packet(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
      std::fill<typename std::array<uint8_t, 5>::iterator, uint8_t>(this->address.begin(), this->address.end(), 0);
      this->datarate = 0;
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
  Type & set__packet(
    const crtp_interface::msg::CrtpPacket_<ContainerAllocator> & _arg)
  {
    this->packet = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crtp_interface__srv__CrtpPacketSend_Request
    std::shared_ptr<crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crtp_interface__srv__CrtpPacketSend_Request
    std::shared_ptr<crtp_interface::srv::CrtpPacketSend_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CrtpPacketSend_Request_ & other) const
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
    if (this->packet != other.packet) {
      return false;
    }
    return true;
  }
  bool operator!=(const CrtpPacketSend_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CrtpPacketSend_Request_

// alias to use template instance with default allocator
using CrtpPacketSend_Request =
  crtp_interface::srv::CrtpPacketSend_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crtp_interface


#ifndef _WIN32
# define DEPRECATED__crtp_interface__srv__CrtpPacketSend_Response __attribute__((deprecated))
#else
# define DEPRECATED__crtp_interface__srv__CrtpPacketSend_Response __declspec(deprecated)
#endif

namespace crtp_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CrtpPacketSend_Response_
{
  using Type = CrtpPacketSend_Response_<ContainerAllocator>;

  explicit CrtpPacketSend_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit CrtpPacketSend_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crtp_interface__srv__CrtpPacketSend_Response
    std::shared_ptr<crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crtp_interface__srv__CrtpPacketSend_Response
    std::shared_ptr<crtp_interface::srv::CrtpPacketSend_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CrtpPacketSend_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const CrtpPacketSend_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CrtpPacketSend_Response_

// alias to use template instance with default allocator
using CrtpPacketSend_Response =
  crtp_interface::srv::CrtpPacketSend_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crtp_interface

namespace crtp_interface
{

namespace srv
{

struct CrtpPacketSend
{
  using Request = crtp_interface::srv::CrtpPacketSend_Request;
  using Response = crtp_interface::srv::CrtpPacketSend_Response;
};

}  // namespace srv

}  // namespace crtp_interface

#endif  // CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__STRUCT_HPP_
