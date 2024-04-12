// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from crtp_interface:msg/SetAutoping.idl
// generated code does not contain a copyright notice
#include "crtp_interface/msg/detail/set_autoping__rosidl_typesupport_fastrtps_cpp.hpp"
#include "crtp_interface/msg/detail/set_autoping__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace crtp_interface
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crtp_interface
cdr_serialize(
  const crtp_interface::msg::SetAutoping & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: channel
  cdr << ros_message.channel;
  // Member: address
  {
    cdr << ros_message.address;
  }
  // Member: datarate
  cdr << ros_message.datarate;
  // Member: rate
  cdr << ros_message.rate;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crtp_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  crtp_interface::msg::SetAutoping & ros_message)
{
  // Member: channel
  cdr >> ros_message.channel;

  // Member: address
  {
    cdr >> ros_message.address;
  }

  // Member: datarate
  cdr >> ros_message.datarate;

  // Member: rate
  cdr >> ros_message.rate;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crtp_interface
get_serialized_size(
  const crtp_interface::msg::SetAutoping & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: channel
  {
    size_t item_size = sizeof(ros_message.channel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: address
  {
    size_t array_size = 5;
    size_t item_size = sizeof(ros_message.address[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: datarate
  {
    size_t item_size = sizeof(ros_message.datarate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rate
  {
    size_t item_size = sizeof(ros_message.rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crtp_interface
max_serialized_size_SetAutoping(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: channel
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: address
  {
    size_t array_size = 5;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: datarate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = crtp_interface::msg::SetAutoping;
    is_plain =
      (
      offsetof(DataType, rate) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SetAutoping__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const crtp_interface::msg::SetAutoping *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SetAutoping__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<crtp_interface::msg::SetAutoping *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SetAutoping__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const crtp_interface::msg::SetAutoping *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SetAutoping__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SetAutoping(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SetAutoping__callbacks = {
  "crtp_interface::msg",
  "SetAutoping",
  _SetAutoping__cdr_serialize,
  _SetAutoping__cdr_deserialize,
  _SetAutoping__get_serialized_size,
  _SetAutoping__max_serialized_size
};

static rosidl_message_type_support_t _SetAutoping__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetAutoping__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace crtp_interface

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_crtp_interface
const rosidl_message_type_support_t *
get_message_type_support_handle<crtp_interface::msg::SetAutoping>()
{
  return &crtp_interface::msg::typesupport_fastrtps_cpp::_SetAutoping__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, crtp_interface, msg, SetAutoping)() {
  return &crtp_interface::msg::typesupport_fastrtps_cpp::_SetAutoping__handle;
}

#ifdef __cplusplus
}
#endif
