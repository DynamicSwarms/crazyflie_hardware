// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from crtp_interface:msg/CrtpPacket.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "crtp_interface/msg/detail/crtp_packet__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace crtp_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CrtpPacket_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crtp_interface::msg::CrtpPacket(_init);
}

void CrtpPacket_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crtp_interface::msg::CrtpPacket *>(message_memory);
  typed_message->~CrtpPacket();
}

size_t size_function__CrtpPacket__data(const void * untyped_member)
{
  (void)untyped_member;
  return 31;
}

const void * get_const_function__CrtpPacket__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 31> *>(untyped_member);
  return &member[index];
}

void * get_function__CrtpPacket__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 31> *>(untyped_member);
  return &member[index];
}

void fetch_function__CrtpPacket__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__CrtpPacket__data(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__CrtpPacket__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__CrtpPacket__data(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CrtpPacket_message_member_array[4] = {
  {
    "port",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::CrtpPacket, port),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "channel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::CrtpPacket, channel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    31,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::CrtpPacket, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__CrtpPacket__data,  // size() function pointer
    get_const_function__CrtpPacket__data,  // get_const(index) function pointer
    get_function__CrtpPacket__data,  // get(index) function pointer
    fetch_function__CrtpPacket__data,  // fetch(index, &value) function pointer
    assign_function__CrtpPacket__data,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "data_length",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::CrtpPacket, data_length),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CrtpPacket_message_members = {
  "crtp_interface::msg",  // message namespace
  "CrtpPacket",  // message name
  4,  // number of fields
  sizeof(crtp_interface::msg::CrtpPacket),
  CrtpPacket_message_member_array,  // message members
  CrtpPacket_init_function,  // function to initialize message memory (memory has to be allocated)
  CrtpPacket_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CrtpPacket_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CrtpPacket_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace crtp_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<crtp_interface::msg::CrtpPacket>()
{
  return &::crtp_interface::msg::rosidl_typesupport_introspection_cpp::CrtpPacket_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crtp_interface, msg, CrtpPacket)() {
  return &::crtp_interface::msg::rosidl_typesupport_introspection_cpp::CrtpPacket_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
