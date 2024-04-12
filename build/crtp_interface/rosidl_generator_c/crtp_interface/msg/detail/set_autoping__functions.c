// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crtp_interface:msg/SetAutoping.idl
// generated code does not contain a copyright notice
#include "crtp_interface/msg/detail/set_autoping__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
crtp_interface__msg__SetAutoping__init(crtp_interface__msg__SetAutoping * msg)
{
  if (!msg) {
    return false;
  }
  // channel
  // address
  // datarate
  // rate
  return true;
}

void
crtp_interface__msg__SetAutoping__fini(crtp_interface__msg__SetAutoping * msg)
{
  if (!msg) {
    return;
  }
  // channel
  // address
  // datarate
  // rate
}

bool
crtp_interface__msg__SetAutoping__are_equal(const crtp_interface__msg__SetAutoping * lhs, const crtp_interface__msg__SetAutoping * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // channel
  if (lhs->channel != rhs->channel) {
    return false;
  }
  // address
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->address[i] != rhs->address[i]) {
      return false;
    }
  }
  // datarate
  if (lhs->datarate != rhs->datarate) {
    return false;
  }
  // rate
  if (lhs->rate != rhs->rate) {
    return false;
  }
  return true;
}

bool
crtp_interface__msg__SetAutoping__copy(
  const crtp_interface__msg__SetAutoping * input,
  crtp_interface__msg__SetAutoping * output)
{
  if (!input || !output) {
    return false;
  }
  // channel
  output->channel = input->channel;
  // address
  for (size_t i = 0; i < 5; ++i) {
    output->address[i] = input->address[i];
  }
  // datarate
  output->datarate = input->datarate;
  // rate
  output->rate = input->rate;
  return true;
}

crtp_interface__msg__SetAutoping *
crtp_interface__msg__SetAutoping__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__msg__SetAutoping * msg = (crtp_interface__msg__SetAutoping *)allocator.allocate(sizeof(crtp_interface__msg__SetAutoping), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crtp_interface__msg__SetAutoping));
  bool success = crtp_interface__msg__SetAutoping__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crtp_interface__msg__SetAutoping__destroy(crtp_interface__msg__SetAutoping * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crtp_interface__msg__SetAutoping__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crtp_interface__msg__SetAutoping__Sequence__init(crtp_interface__msg__SetAutoping__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__msg__SetAutoping * data = NULL;

  if (size) {
    data = (crtp_interface__msg__SetAutoping *)allocator.zero_allocate(size, sizeof(crtp_interface__msg__SetAutoping), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crtp_interface__msg__SetAutoping__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crtp_interface__msg__SetAutoping__fini(&data[i - 1]);
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
crtp_interface__msg__SetAutoping__Sequence__fini(crtp_interface__msg__SetAutoping__Sequence * array)
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
      crtp_interface__msg__SetAutoping__fini(&array->data[i]);
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

crtp_interface__msg__SetAutoping__Sequence *
crtp_interface__msg__SetAutoping__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__msg__SetAutoping__Sequence * array = (crtp_interface__msg__SetAutoping__Sequence *)allocator.allocate(sizeof(crtp_interface__msg__SetAutoping__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crtp_interface__msg__SetAutoping__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crtp_interface__msg__SetAutoping__Sequence__destroy(crtp_interface__msg__SetAutoping__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crtp_interface__msg__SetAutoping__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crtp_interface__msg__SetAutoping__Sequence__are_equal(const crtp_interface__msg__SetAutoping__Sequence * lhs, const crtp_interface__msg__SetAutoping__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crtp_interface__msg__SetAutoping__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crtp_interface__msg__SetAutoping__Sequence__copy(
  const crtp_interface__msg__SetAutoping__Sequence * input,
  crtp_interface__msg__SetAutoping__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crtp_interface__msg__SetAutoping);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crtp_interface__msg__SetAutoping * data =
      (crtp_interface__msg__SetAutoping *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crtp_interface__msg__SetAutoping__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crtp_interface__msg__SetAutoping__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crtp_interface__msg__SetAutoping__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
