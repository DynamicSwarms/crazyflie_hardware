// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crtp_interface:srv/CrtpPacketSend.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__STRUCT_H_
#define CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'packet'
#include "crtp_interface/msg/detail/crtp_packet__struct.h"

/// Struct defined in srv/CrtpPacketSend in the package crtp_interface.
typedef struct crtp_interface__srv__CrtpPacketSend_Request
{
  uint8_t channel;
  uint8_t address[5];
  uint8_t datarate;
  crtp_interface__msg__CrtpPacket packet;
} crtp_interface__srv__CrtpPacketSend_Request;

// Struct for a sequence of crtp_interface__srv__CrtpPacketSend_Request.
typedef struct crtp_interface__srv__CrtpPacketSend_Request__Sequence
{
  crtp_interface__srv__CrtpPacketSend_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crtp_interface__srv__CrtpPacketSend_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CrtpPacketSend in the package crtp_interface.
typedef struct crtp_interface__srv__CrtpPacketSend_Response
{
  bool success;
} crtp_interface__srv__CrtpPacketSend_Response;

// Struct for a sequence of crtp_interface__srv__CrtpPacketSend_Response.
typedef struct crtp_interface__srv__CrtpPacketSend_Response__Sequence
{
  crtp_interface__srv__CrtpPacketSend_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crtp_interface__srv__CrtpPacketSend_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__STRUCT_H_
