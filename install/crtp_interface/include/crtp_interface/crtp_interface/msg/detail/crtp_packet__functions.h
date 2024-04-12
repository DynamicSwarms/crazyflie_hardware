// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from crtp_interface:msg/CrtpPacket.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__FUNCTIONS_H_
#define CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "crtp_interface/msg/rosidl_generator_c__visibility_control.h"

#include "crtp_interface/msg/detail/crtp_packet__struct.h"

/// Initialize msg/CrtpPacket message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * crtp_interface__msg__CrtpPacket
 * )) before or use
 * crtp_interface__msg__CrtpPacket__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
bool
crtp_interface__msg__CrtpPacket__init(crtp_interface__msg__CrtpPacket * msg);

/// Finalize msg/CrtpPacket message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
void
crtp_interface__msg__CrtpPacket__fini(crtp_interface__msg__CrtpPacket * msg);

/// Create msg/CrtpPacket message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * crtp_interface__msg__CrtpPacket__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
crtp_interface__msg__CrtpPacket *
crtp_interface__msg__CrtpPacket__create();

/// Destroy msg/CrtpPacket message.
/**
 * It calls
 * crtp_interface__msg__CrtpPacket__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
void
crtp_interface__msg__CrtpPacket__destroy(crtp_interface__msg__CrtpPacket * msg);

/// Check for msg/CrtpPacket message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
bool
crtp_interface__msg__CrtpPacket__are_equal(const crtp_interface__msg__CrtpPacket * lhs, const crtp_interface__msg__CrtpPacket * rhs);

/// Copy a msg/CrtpPacket message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
bool
crtp_interface__msg__CrtpPacket__copy(
  const crtp_interface__msg__CrtpPacket * input,
  crtp_interface__msg__CrtpPacket * output);

/// Initialize array of msg/CrtpPacket messages.
/**
 * It allocates the memory for the number of elements and calls
 * crtp_interface__msg__CrtpPacket__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
bool
crtp_interface__msg__CrtpPacket__Sequence__init(crtp_interface__msg__CrtpPacket__Sequence * array, size_t size);

/// Finalize array of msg/CrtpPacket messages.
/**
 * It calls
 * crtp_interface__msg__CrtpPacket__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
void
crtp_interface__msg__CrtpPacket__Sequence__fini(crtp_interface__msg__CrtpPacket__Sequence * array);

/// Create array of msg/CrtpPacket messages.
/**
 * It allocates the memory for the array and calls
 * crtp_interface__msg__CrtpPacket__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
crtp_interface__msg__CrtpPacket__Sequence *
crtp_interface__msg__CrtpPacket__Sequence__create(size_t size);

/// Destroy array of msg/CrtpPacket messages.
/**
 * It calls
 * crtp_interface__msg__CrtpPacket__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
void
crtp_interface__msg__CrtpPacket__Sequence__destroy(crtp_interface__msg__CrtpPacket__Sequence * array);

/// Check for msg/CrtpPacket message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
bool
crtp_interface__msg__CrtpPacket__Sequence__are_equal(const crtp_interface__msg__CrtpPacket__Sequence * lhs, const crtp_interface__msg__CrtpPacket__Sequence * rhs);

/// Copy an array of msg/CrtpPacket messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_crtp_interface
bool
crtp_interface__msg__CrtpPacket__Sequence__copy(
  const crtp_interface__msg__CrtpPacket__Sequence * input,
  crtp_interface__msg__CrtpPacket__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__FUNCTIONS_H_
