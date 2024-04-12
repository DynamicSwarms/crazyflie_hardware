// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from crtp_interface:msg/SetAutoping.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "crtp_interface/msg/detail/set_autoping__struct.hpp"
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

void SetAutoping_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crtp_interface::msg::SetAutoping(_init);
}

void SetAutoping_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crtp_interface::msg::SetAutoping *>(message_memory);
  typed_message->~SetAutoping();
}

size_t size_function__SetAutoping__address(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__SetAutoping__address(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__SetAutoping__address(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetAutoping__address(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__SetAutoping__address(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__SetAutoping__address(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__SetAutoping__address(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetAutoping_message_member_array[4] = {
  {
    "channel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::SetAutoping, channel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "address",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::SetAutoping, address),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetAutoping__address,  // size() function pointer
    get_const_function__SetAutoping__address,  // get_const(index) function pointer
    get_function__SetAutoping__address,  // get(index) function pointer
    fetch_function__SetAutoping__address,  // fetch(index, &value) function pointer
    assign_function__SetAutoping__address,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "datarate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::SetAutoping, datarate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::SetAutoping, rate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetAutoping_message_members = {
  "crtp_interface::msg",  // message namespace
  "SetAutoping",  // message name
  4,  // number of fields
  sizeof(crtp_interface::msg::SetAutoping),
  SetAutoping_message_member_array,  // message members
  SetAutoping_init_function,  // function to initialize message memory (memory has to be allocated)
  SetAutoping_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetAutoping_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetAutoping_message_members,
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
get_message_type_support_handle<crtp_interface::msg::SetAutoping>()
{
  return &::crtp_interface::msg::rosidl_typesupport_introspection_cpp::SetAutoping_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crtp_interface, msg, SetAutoping)() {
  return &::crtp_interface::msg::rosidl_typesupport_introspection_cpp::SetAutoping_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
