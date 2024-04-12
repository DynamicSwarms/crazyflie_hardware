// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from crtp_interface:srv/CrtpPacketSend.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "crtp_interface/srv/detail/crtp_packet_send__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace crtp_interface
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CrtpPacketSend_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crtp_interface::srv::CrtpPacketSend_Request(_init);
}

void CrtpPacketSend_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crtp_interface::srv::CrtpPacketSend_Request *>(message_memory);
  typed_message->~CrtpPacketSend_Request();
}

size_t size_function__CrtpPacketSend_Request__address(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__CrtpPacketSend_Request__address(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__CrtpPacketSend_Request__address(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 5> *>(untyped_member);
  return &member[index];
}

void fetch_function__CrtpPacketSend_Request__address(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__CrtpPacketSend_Request__address(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__CrtpPacketSend_Request__address(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__CrtpPacketSend_Request__address(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CrtpPacketSend_Request_message_member_array[4] = {
  {
    "channel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::srv::CrtpPacketSend_Request, channel),  // bytes offset in struct
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
    offsetof(crtp_interface::srv::CrtpPacketSend_Request, address),  // bytes offset in struct
    nullptr,  // default value
    size_function__CrtpPacketSend_Request__address,  // size() function pointer
    get_const_function__CrtpPacketSend_Request__address,  // get_const(index) function pointer
    get_function__CrtpPacketSend_Request__address,  // get(index) function pointer
    fetch_function__CrtpPacketSend_Request__address,  // fetch(index, &value) function pointer
    assign_function__CrtpPacketSend_Request__address,  // assign(index, value) function pointer
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
    offsetof(crtp_interface::srv::CrtpPacketSend_Request, datarate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "packet",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<crtp_interface::msg::CrtpPacket>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::srv::CrtpPacketSend_Request, packet),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CrtpPacketSend_Request_message_members = {
  "crtp_interface::srv",  // message namespace
  "CrtpPacketSend_Request",  // message name
  4,  // number of fields
  sizeof(crtp_interface::srv::CrtpPacketSend_Request),
  CrtpPacketSend_Request_message_member_array,  // message members
  CrtpPacketSend_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  CrtpPacketSend_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CrtpPacketSend_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CrtpPacketSend_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace crtp_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<crtp_interface::srv::CrtpPacketSend_Request>()
{
  return &::crtp_interface::srv::rosidl_typesupport_introspection_cpp::CrtpPacketSend_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crtp_interface, srv, CrtpPacketSend_Request)() {
  return &::crtp_interface::srv::rosidl_typesupport_introspection_cpp::CrtpPacketSend_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "crtp_interface/srv/detail/crtp_packet_send__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace crtp_interface
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CrtpPacketSend_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crtp_interface::srv::CrtpPacketSend_Response(_init);
}

void CrtpPacketSend_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crtp_interface::srv::CrtpPacketSend_Response *>(message_memory);
  typed_message->~CrtpPacketSend_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CrtpPacketSend_Response_message_member_array[1] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crtp_interface::srv::CrtpPacketSend_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CrtpPacketSend_Response_message_members = {
  "crtp_interface::srv",  // message namespace
  "CrtpPacketSend_Response",  // message name
  1,  // number of fields
  sizeof(crtp_interface::srv::CrtpPacketSend_Response),
  CrtpPacketSend_Response_message_member_array,  // message members
  CrtpPacketSend_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  CrtpPacketSend_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CrtpPacketSend_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CrtpPacketSend_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace crtp_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<crtp_interface::srv::CrtpPacketSend_Response>()
{
  return &::crtp_interface::srv::rosidl_typesupport_introspection_cpp::CrtpPacketSend_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crtp_interface, srv, CrtpPacketSend_Response)() {
  return &::crtp_interface::srv::rosidl_typesupport_introspection_cpp::CrtpPacketSend_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "crtp_interface/srv/detail/crtp_packet_send__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace crtp_interface
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers CrtpPacketSend_service_members = {
  "crtp_interface::srv",  // service namespace
  "CrtpPacketSend",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<crtp_interface::srv::CrtpPacketSend>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t CrtpPacketSend_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CrtpPacketSend_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace crtp_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<crtp_interface::srv::CrtpPacketSend>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::crtp_interface::srv::rosidl_typesupport_introspection_cpp::CrtpPacketSend_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::crtp_interface::srv::CrtpPacketSend_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::crtp_interface::srv::CrtpPacketSend_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crtp_interface, srv, CrtpPacketSend)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<crtp_interface::srv::CrtpPacketSend>();
}

#ifdef __cplusplus
}
#endif
