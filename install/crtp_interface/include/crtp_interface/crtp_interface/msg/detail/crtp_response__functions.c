// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crtp_interface:msg/CrtpResponse.idl
// generated code does not contain a copyright notice
#include "crtp_interface/msg/detail/crtp_response__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `address`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `packet`
#include "crtp_interface/msg/detail/crtp_packet__functions.h"

bool
crtp_interface__msg__CrtpResponse__init(crtp_interface__msg__CrtpResponse * msg)
{
  if (!msg) {
    return false;
  }
  // channel
  // address
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->address, 0)) {
    crtp_interface__msg__CrtpResponse__fini(msg);
    return false;
  }
  // packet
  if (!crtp_interface__msg__CrtpPacket__init(&msg->packet)) {
    crtp_interface__msg__CrtpResponse__fini(msg);
    return false;
  }
  return true;
}

void
crtp_interface__msg__CrtpResponse__fini(crtp_interface__msg__CrtpResponse * msg)
{
  if (!msg) {
    return;
  }
  // channel
  // address
  rosidl_runtime_c__uint8__Sequence__fini(&msg->address);
  // packet
  crtp_interface__msg__CrtpPacket__fini(&msg->packet);
}

bool
crtp_interface__msg__CrtpResponse__are_equal(const crtp_interface__msg__CrtpResponse * lhs, const crtp_interface__msg__CrtpResponse * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // channel
  if (lhs->channel != rhs->channel) {
    return false;
  }
  // address
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->address), &(rhs->address)))
  {
    return false;
  }
  // packet
  if (!crtp_interface__msg__CrtpPacket__are_equal(
      &(lhs->packet), &(rhs->packet)))
  {
    return false;
  }
  return true;
}

bool
crtp_interface__msg__CrtpResponse__copy(
  const crtp_interface__msg__CrtpResponse * input,
  crtp_interface__msg__CrtpResponse * output)
{
  if (!input || !output) {
    return false;
  }
  // channel
  output->channel = input->channel;
  // address
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->address), &(output->address)))
  {
    return false;
  }
  // packet
  if (!crtp_interface__msg__CrtpPacket__copy(
      &(input->packet), &(output->packet)))
  {
    return false;
  }
  return true;
}

crtp_interface__msg__CrtpResponse *
crtp_interface__msg__CrtpResponse__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__msg__CrtpResponse * msg = (crtp_interface__msg__CrtpResponse *)allocator.allocate(sizeof(crtp_interface__msg__CrtpResponse), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crtp_interface__msg__CrtpResponse));
  bool success = crtp_interface__msg__CrtpResponse__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crtp_interface__msg__CrtpResponse__destroy(crtp_interface__msg__CrtpResponse * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crtp_interface__msg__CrtpResponse__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crtp_interface__msg__CrtpResponse__Sequence__init(crtp_interface__msg__CrtpResponse__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__msg__CrtpResponse * data = NULL;

  if (size) {
    data = (crtp_interface__msg__CrtpResponse *)allocator.zero_allocate(size, sizeof(crtp_interface__msg__CrtpResponse), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crtp_interface__msg__CrtpResponse__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crtp_interface__msg__CrtpResponse__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crtp_interface__msg__CrtpResponse__Sequence__fini(crtp_interface__msg__CrtpResponse__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crtp_interface__msg__CrtpResponse__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crtp_interface__msg__CrtpResponse__Sequence *
crtp_interface__msg__CrtpResponse__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__msg__CrtpResponse__Sequence * array = (crtp_interface__msg__CrtpResponse__Sequence *)allocator.allocate(sizeof(crtp_interface__msg__CrtpResponse__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crtp_interface__msg__CrtpResponse__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crtp_interface__msg__CrtpResponse__Sequence__destroy(crtp_interface__msg__CrtpResponse__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crtp_interface__msg__CrtpResponse__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crtp_interface__msg__CrtpResponse__Sequence__are_equal(const crtp_interface__msg__CrtpResponse__Sequence * lhs, const crtp_interface__msg__CrtpResponse__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crtp_interface__msg__CrtpResponse__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crtp_interface__msg__CrtpResponse__Sequence__copy(
  const crtp_interface__msg__CrtpResponse__Sequence * input,
  crtp_interface__msg__CrtpResponse__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crtp_interface__msg__CrtpResponse);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crtp_interface__msg__CrtpResponse * data =
      (crtp_interface__msg__CrtpResponse *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crtp_interface__msg__CrtpResponse__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crtp_interface__msg__CrtpResponse__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crtp_interface__msg__CrtpResponse__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
