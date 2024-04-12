// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crtp_interface:msg/CrtpPacket.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__STRUCT_H_
#define CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CrtpPacket in the package crtp_interface.
typedef struct crtp_interface__msg__CrtpPacket
{
  uint8_t port;
  uint8_t channel;
  uint8_t data[31];
  uint8_t data_length;
} crtp_interface__msg__CrtpPacket;

// Struct for a sequence of crtp_interface__msg__CrtpPacket.
typedef struct crtp_interface__msg__CrtpPacket__Sequence
{
  crtp_interface__msg__CrtpPacket * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crtp_interface__msg__CrtpPacket__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__STRUCT_H_
