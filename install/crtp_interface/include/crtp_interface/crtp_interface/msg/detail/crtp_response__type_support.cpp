// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from crtp_interface:msg/CrtpResponse.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "crtp_interface/msg/detail/crtp_response__struct.hpp"
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

void CrtpResponse_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crtp_interface::msg::CrtpResponse(_init);
}

void CrtpResponse_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crtp_interface::msg::CrtpResponse *>(message_memory);
  typed_message->~CrtpResponse();
}

size_t size_function__CrtpResponse__address(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CrtpResponse__address(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__CrtpResponse__address(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__CrtpResponse__address(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__CrtpResponse__address(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__CrtpResponse__address(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__CrtpResponse__address(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__CrtpResponse__address(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CrtpResponse_message_member_array[3] = {
  {
    "channel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::CrtpResponse, channel),  // bytes offset in struct
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
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::CrtpResponse, address),  // bytes offset in struct
    nullptr,  // default value
    size_function__CrtpResponse__address,  // size() function pointer
    get_const_function__CrtpResponse__address,  // get_const(index) function pointer
    get_function__CrtpResponse__address,  // get(index) function pointer
    fetch_function__CrtpResponse__address,  // fetch(index, &value) function pointer
    assign_function__CrtpResponse__address,  // assign(index, value) function pointer
    resize_function__CrtpResponse__address  // resize(index) function pointer
  },
  {
    "packet",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<crtp_interface::msg::CrtpPacket>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::msg::CrtpResponse, packet),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CrtpResponse_message_members = {
  "crtp_interface::msg",  // message namespace
  "CrtpResponse",  // message name
  3,  // number of fields
  sizeof(crtp_interface::msg::CrtpResponse),
  CrtpResponse_message_member_array,  // message members
  CrtpResponse_init_function,  // function to initialize message memory (memory has to be allocated)
  CrtpResponse_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CrtpResponse_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CrtpResponse_message_members,
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
get_message_type_support_handle<crtp_interface::msg::CrtpResponse>()
{
  return &::crtp_interface::msg::rosidl_typesupport_introspection_cpp::CrtpResponse_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crtp_interface, msg, CrtpResponse)() {
  return &::crtp_interface::msg::rosidl_typesupport_introspection_cpp::CrtpResponse_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
