// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crtp_interface:srv/CrtpPacketSend.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crtp_interface/srv/detail/crtp_packet_send__rosidl_typesupport_introspection_c.h"
#include "crtp_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crtp_interface/srv/detail/crtp_packet_send__functions.h"
#include "crtp_interface/srv/detail/crtp_packet_send__struct.h"


// Include directives for member types
// Member `packet`
#include "crtp_interface/msg/crtp_packet.h"
// Member `packet`
#include "crtp_interface/msg/detail/crtp_packet__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crtp_interface__srv__CrtpPacketSend_Request__init(message_memory);
}

void crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_fini_function(void * message_memory)
{
  crtp_interface__srv__CrtpPacketSend_Request__fini(message_memory);
}

size_t crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__size_function__CrtpPacketSend_Request__address(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__get_const_function__CrtpPacketSend_Request__address(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__get_function__CrtpPacketSend_Request__address(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__fetch_function__CrtpPacketSend_Request__address(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__get_const_function__CrtpPacketSend_Request__address(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__assign_function__CrtpPacketSend_Request__address(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__get_function__CrtpPacketSend_Request__address(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_member_array[4] = {
  {
    "channel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__srv__CrtpPacketSend_Request, channel),  // bytes offset in struct
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
    offsetof(crtp_interface__srv__CrtpPacketSend_Request, address),  // bytes offset in struct
    NULL,  // default value
    crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__size_function__CrtpPacketSend_Request__address,  // size() function pointer
    crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__get_const_function__CrtpPacketSend_Request__address,  // get_const(index) function pointer
    crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__get_function__CrtpPacketSend_Request__address,  // get(index) function pointer
    crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__fetch_function__CrtpPacketSend_Request__address,  // fetch(index, &value) function pointer
    crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__assign_function__CrtpPacketSend_Request__address,  // assign(index, value) function pointer
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
    offsetof(crtp_interface__srv__CrtpPacketSend_Request, datarate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "packet",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__srv__CrtpPacketSend_Request, packet),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_members = {
  "crtp_interface__srv",  // message namespace
  "CrtpPacketSend_Request",  // message name
  4,  // number of fields
  sizeof(crtp_interface__srv__CrtpPacketSend_Request),
  crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_member_array,  // message members
  crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_type_support_handle = {
  0,
  &crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crtp_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, srv, CrtpPacketSend_Request)() {
  crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, msg, CrtpPacket)();
  if (!crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_type_support_handle.typesupport_identifier) {
    crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crtp_interface__srv__CrtpPacketSend_Request__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "crtp_interface/srv/detail/crtp_packet_send__rosidl_typesupport_introspection_c.h"
// already included above
// #include "crtp_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "crtp_interface/srv/detail/crtp_packet_send__functions.h"
// already included above
// #include "crtp_interface/srv/detail/crtp_packet_send__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crtp_interface__srv__CrtpPacketSend_Response__init(message_memory);
}

void crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_fini_function(void * message_memory)
{
  crtp_interface__srv__CrtpPacketSend_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface__srv__CrtpPacketSend_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_message_members = {
  "crtp_interface__srv",  // message namespace
  "CrtpPacketSend_Response",  // message name
  1,  // number of fields
  sizeof(crtp_interface__srv__CrtpPacketSend_Response),
  crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_message_member_array,  // message members
  crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_message_type_support_handle = {
  0,
  &crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crtp_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, srv, CrtpPacketSend_Response)() {
  if (!crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_message_type_support_handle.typesupport_identifier) {
    crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crtp_interface__srv__CrtpPacketSend_Response__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "crtp_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "crtp_interface/srv/detail/crtp_packet_send__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers crtp_interface__srv__detail__crtp_packet_send__rosidl_typesupport_introspection_c__CrtpPacketSend_service_members = {
  "crtp_interface__srv",  // service namespace
  "CrtpPacketSend",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // crtp_interface__srv__detail__crtp_packet_send__rosidl_typesupport_introspection_c__CrtpPacketSend_Request_message_type_support_handle,
  NULL  // response message
  // crtp_interface__srv__detail__crtp_packet_send__rosidl_typesupport_introspection_c__CrtpPacketSend_Response_message_type_support_handle
};

static rosidl_service_type_support_t crtp_interface__srv__detail__crtp_packet_send__rosidl_typesupport_introspection_c__CrtpPacketSend_service_type_support_handle = {
  0,
  &crtp_interface__srv__detail__crtp_packet_send__rosidl_typesupport_introspection_c__CrtpPacketSend_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, srv, CrtpPacketSend_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, srv, CrtpPacketSend_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crtp_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, srv, CrtpPacketSend)() {
  if (!crtp_interface__srv__detail__crtp_packet_send__rosidl_typesupport_introspection_c__CrtpPacketSend_service_type_support_handle.typesupport_identifier) {
    crtp_interface__srv__detail__crtp_packet_send__rosidl_typesupport_introspection_c__CrtpPacketSend_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)crtp_interface__srv__detail__crtp_packet_send__rosidl_typesupport_introspection_c__CrtpPacketSend_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, srv, CrtpPacketSend_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crtp_interface, srv, CrtpPacketSend_Response)()->data;
  }

  return &crtp_interface__srv__detail__crtp_packet_send__rosidl_typesupport_introspection_c__CrtpPacketSend_service_type_support_handle;
}
