// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crtp_interface:msg/CrtpResponse.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__STRUCT_H_
#define CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'address'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'packet'
#include "crtp_interface/msg/detail/crtp_packet__struct.h"

/// Struct defined in msg/CrtpResponse in the package crtp_interface.
typedef struct crtp_interface__msg__CrtpResponse
{
  uint8_t channel;
  rosidl_runtime_c__uint8__Sequence address;
  crtp_interface__msg__CrtpPacket packet;
} crtp_interface__msg__CrtpResponse;

// Struct for a sequence of crtp_interface__msg__CrtpResponse.
typedef struct crtp_interface__msg__CrtpResponse__Sequence
{
  crtp_interface__msg__CrtpResponse * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crtp_interface__msg__CrtpResponse__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__STRUCT_H_
