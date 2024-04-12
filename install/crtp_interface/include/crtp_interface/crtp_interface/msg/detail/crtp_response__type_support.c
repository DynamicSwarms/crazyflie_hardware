// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crtp_interface:msg/CrtpResponse.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crtp_interface/msg/detail/crtp_response__rosidl_typesupport_introspection_c.h"
#include "crtp_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crtp_interface/msg/detail/crtp_response__functions.h"
#include "crtp_interface/msg/detail/crtp_response__struct.h"


// Include directives for member types
// Member `address`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `packet`
#include "crtp_interface/msg/crtp_packet.h"
// Member `packet`
#include "crtp_interface/msg/detail/crtp_packet__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crtp_interface__msg__CrtpResponse__init(message_memory);
}

void crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_fini_function(void * message_memory)
{
  crtp_interface__msg__CrtpResponse__fini(message_memory);
}

size_t crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__size_function__CrtpResponse__address(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__get_const_function__CrtpResponse__address(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__get_function__CrtpResponse__address(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__fetch_function__CrtpResponse__address(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__get_const_function__CrtpResponse__address(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__assign_function__CrtpResponse__address(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__get_function__CrtpResponse__address(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__resize_function__CrtpResponse__address(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_message_member_array[3] = {
  {
    "channel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__msg__CrtpResponse, channel),  // bytes offset in struct
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
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__msg__CrtpResponse, address),  // bytes offset in struct
    NULL,  // default value
    crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__size_function__CrtpResponse__address,  // size() function pointer
    crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__get_const_function__CrtpResponse__address,  // get_const(index) function pointer
    crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__get_function__CrtpResponse__address,  // get(index) function pointer
    crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__fetch_function__CrtpResponse__address,  // fetch(index, &value) function pointer
    crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__assign_function__CrtpResponse__address,  // assign(index, value) function pointer
    crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__resize_function__CrtpResponse__address  // resize(index) function pointer
  },
  {
    "packet",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__msg__CrtpResponse, packet),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_message_members = {
  "crtp_interface__msg",  // message namespace
  "CrtpResponse",  // message name
  3,  // number of fields
  sizeof(crtp_interface__msg__CrtpResponse),
  crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_message_member_array,  // message members
  crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_init_function,  // function to initialize message memory (memory has to be allocated)
  crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_message_type_support_handle = {
  0,
  &crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crtp_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, msg, CrtpResponse)() {
  crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, msg, CrtpPacket)();
  if (!crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_message_type_support_handle.typesupport_identifier) {
    crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crtp_interface__msg__CrtpResponse__rosidl_typesupport_introspection_c__CrtpResponse_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
