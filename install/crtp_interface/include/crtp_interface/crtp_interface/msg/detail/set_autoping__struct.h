// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crtp_interface:msg/SetAutoping.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__STRUCT_H_
#define CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SetAutoping in the package crtp_interface.
typedef struct crtp_interface__msg__SetAutoping
{
  uint8_t channel;
  uint8_t address[5];
  uint8_t datarate;
  uint16_t rate;
} crtp_interface__msg__SetAutoping;

// Struct for a sequence of crtp_interface__msg__SetAutoping.
typedef struct crtp_interface__msg__SetAutoping__Sequence
{
  crtp_interface__msg__SetAutoping * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crtp_interface__msg__SetAutoping__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__STRUCT_H_
