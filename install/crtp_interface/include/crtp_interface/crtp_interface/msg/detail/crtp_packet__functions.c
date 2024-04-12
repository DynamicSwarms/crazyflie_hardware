// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crtp_interface:msg/CrtpPacket.idl
// generated code does not contain a copyright notice
#include "crtp_interface/msg/detail/crtp_packet__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
crtp_interface__msg__CrtpPacket__init(crtp_interface__msg__CrtpPacket * msg)
{
  if (!msg) {
    return false;
  }
  // port
  // channel
  // data
  // data_length
  return true;
}

void
crtp_interface__msg__CrtpPacket__fini(crtp_interface__msg__CrtpPacket * msg)
{
  if (!msg) {
    return;
  }
  // port
  // channel
  // data
  // data_length
}

bool
crtp_interface__msg__CrtpPacket__are_equal(const crtp_interface__msg__CrtpPacket * lhs, const crtp_interface__msg__CrtpPacket * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // port
  if (lhs->port != rhs->port) {
    return false;
  }
  // channel
  if (lhs->channel != rhs->channel) {
    return false;
  }
  // data
  for (size_t i = 0; i < 31; ++i) {
    if (lhs->data[i] != rhs->data[i]) {
      return false;
    }
  }
  // data_length
  if (lhs->data_length != rhs->data_length) {
    return false;
  }
  return true;
}

bool
crtp_interface__msg__CrtpPacket__copy(
  const crtp_interface__msg__CrtpPacket * input,
  crtp_interface__msg__CrtpPacket * output)
{
  if (!input || !output) {
    return false;
  }
  // port
  output->port = input->port;
  // channel
  output->channel = input->channel;
  // data
  for (size_t i = 0; i < 31; ++i) {
    output->data[i] = input->data[i];
  }
  // data_length
  output->data_length = input->data_length;
  return true;
}

crtp_interface__msg__CrtpPacket *
crtp_interface__msg__CrtpPacket__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__msg__CrtpPacket * msg = (crtp_interface__msg__CrtpPacket *)allocator.allocate(sizeof(crtp_interface__msg__CrtpPacket), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crtp_interface__msg__CrtpPacket));
  bool success = crtp_interface__msg__CrtpPacket__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crtp_interface__msg__CrtpPacket__destroy(crtp_interface__msg__CrtpPacket * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crtp_interface__msg__CrtpPacket__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crtp_interface__msg__CrtpPacket__Sequence__init(crtp_interface__msg__CrtpPacket__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__msg__CrtpPacket * data = NULL;

  if (size) {
    data = (crtp_interface__msg__CrtpPacket *)allocator.zero_allocate(size, sizeof(crtp_interface__msg__CrtpPacket), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crtp_interface__msg__CrtpPacket__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crtp_interface__msg__CrtpPacket__fini(&data[i - 1]);
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
crtp_interface__msg__CrtpPacket__Sequence__fini(crtp_interface__msg__CrtpPacket__Sequence * array)
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
      crtp_interface__msg__CrtpPacket__fini(&array->data[i]);
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

crtp_interface__msg__CrtpPacket__Sequence *
crtp_interface__msg__CrtpPacket__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__msg__CrtpPacket__Sequence * array = (crtp_interface__msg__CrtpPacket__Sequence *)allocator.allocate(sizeof(crtp_interface__msg__CrtpPacket__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crtp_interface__msg__CrtpPacket__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crtp_interface__msg__CrtpPacket__Sequence__destroy(crtp_interface__msg__CrtpPacket__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crtp_interface__msg__CrtpPacket__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crtp_interface__msg__CrtpPacket__Sequence__are_equal(const crtp_interface__msg__CrtpPacket__Sequence * lhs, const crtp_interface__msg__CrtpPacket__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crtp_interface__msg__CrtpPacket__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crtp_interface__msg__CrtpPacket__Sequence__copy(
  const crtp_interface__msg__CrtpPacket__Sequence * input,
  crtp_interface__msg__CrtpPacket__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crtp_interface__msg__CrtpPacket);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crtp_interface__msg__CrtpPacket * data =
      (crtp_interface__msg__CrtpPacket *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crtp_interface__msg__CrtpPacket__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crtp_interface__msg__CrtpPacket__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crtp_interface__msg__CrtpPacket__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
