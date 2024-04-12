// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crtp_interface:msg/SetAutoping.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crtp_interface/msg/detail/set_autoping__rosidl_typesupport_introspection_c.h"
#include "crtp_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crtp_interface/msg/detail/set_autoping__functions.h"
#include "crtp_interface/msg/detail/set_autoping__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crtp_interface__msg__SetAutoping__init(message_memory);
}

void crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_fini_function(void * message_memory)
{
  crtp_interface__msg__SetAutoping__fini(message_memory);
}

size_t crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__size_function__SetAutoping__address(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__get_const_function__SetAutoping__address(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__get_function__SetAutoping__address(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__fetch_function__SetAutoping__address(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__get_const_function__SetAutoping__address(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__assign_function__SetAutoping__address(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__get_function__SetAutoping__address(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_message_member_array[4] = {
  {
    "channel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__msg__SetAutoping, channel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "address",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__msg__SetAutoping, address),  // bytes offset in struct
    NULL,  // default value
    crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__size_function__SetAutoping__address,  // size() function pointer
    crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__get_const_function__SetAutoping__address,  // get_const(index) function pointer
    crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__get_function__SetAutoping__address,  // get(index) function pointer
    crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__fetch_function__SetAutoping__address,  // fetch(index, &value) function pointer
    crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__assign_function__SetAutoping__address,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "datarate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__msg__SetAutoping, datarate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__msg__SetAutoping, rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_message_members = {
  "crtp_interface__msg",  // message namespace
  "SetAutoping",  // message name
  4,  // number of fields
  sizeof(crtp_interface__msg__SetAutoping),
  crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_message_member_array,  // message members
  crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_init_function,  // function to initialize message memory (memory has to be allocated)
  crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_message_type_support_handle = {
  0,
  &crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crtp_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, msg, SetAutoping)() {
  if (!crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_message_type_support_handle.typesupport_identifier) {
    crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crtp_interface__msg__SetAutoping__rosidl_typesupport_introspection_c__SetAutoping_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
