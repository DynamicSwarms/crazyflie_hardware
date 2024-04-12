// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crtp_interface:srv/CrtpPacketSend.idl
// generated code does not contain a copyright notice
#include "crtp_interface/srv/detail/crtp_packet_send__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `packet`
#include "crtp_interface/msg/detail/crtp_packet__functions.h"

bool
crtp_interface__srv__CrtpPacketSend_Request__init(crtp_interface__srv__CrtpPacketSend_Request * msg)
{
  if (!msg) {
    return false;
  }
  // channel
  // address
  // datarate
  // packet
  if (!crtp_interface__msg__CrtpPacket__init(&msg->packet)) {
    crtp_interface__srv__CrtpPacketSend_Request__fini(msg);
    return false;
  }
  return true;
}

void
crtp_interface__srv__CrtpPacketSend_Request__fini(crtp_interface__srv__CrtpPacketSend_Request * msg)
{
  if (!msg) {
    return;
  }
  // channel
  // address
  // datarate
  // packet
  crtp_interface__msg__CrtpPacket__fini(&msg->packet);
}

bool
crtp_interface__srv__CrtpPacketSend_Request__are_equal(const crtp_interface__srv__CrtpPacketSend_Request * lhs, const crtp_interface__srv__CrtpPacketSend_Request * rhs)
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
  // packet
  if (!crtp_interface__msg__CrtpPacket__are_equal(
      &(lhs->packet), &(rhs->packet)))
  {
    return false;
  }
  return true;
}

bool
crtp_interface__srv__CrtpPacketSend_Request__copy(
  const crtp_interface__srv__CrtpPacketSend_Request * input,
  crtp_interface__srv__CrtpPacketSend_Request * output)
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
  // packet
  if (!crtp_interface__msg__CrtpPacket__copy(
      &(input->packet), &(output->packet)))
  {
    return false;
  }
  return true;
}

crtp_interface__srv__CrtpPacketSend_Request *
crtp_interface__srv__CrtpPacketSend_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__srv__CrtpPacketSend_Request * msg = (crtp_interface__srv__CrtpPacketSend_Request *)allocator.allocate(sizeof(crtp_interface__srv__CrtpPacketSend_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crtp_interface__srv__CrtpPacketSend_Request));
  bool success = crtp_interface__srv__CrtpPacketSend_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crtp_interface__srv__CrtpPacketSend_Request__destroy(crtp_interface__srv__CrtpPacketSend_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crtp_interface__srv__CrtpPacketSend_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crtp_interface__srv__CrtpPacketSend_Request__Sequence__init(crtp_interface__srv__CrtpPacketSend_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__srv__CrtpPacketSend_Request * data = NULL;

  if (size) {
    data = (crtp_interface__srv__CrtpPacketSend_Request *)allocator.zero_allocate(size, sizeof(crtp_interface__srv__CrtpPacketSend_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crtp_interface__srv__CrtpPacketSend_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crtp_interface__srv__CrtpPacketSend_Request__fini(&data[i - 1]);
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
crtp_interface__srv__CrtpPacketSend_Request__Sequence__fini(crtp_interface__srv__CrtpPacketSend_Request__Sequence * array)
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
      crtp_interface__srv__CrtpPacketSend_Request__fini(&array->data[i]);
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

crtp_interface__srv__CrtpPacketSend_Request__Sequence *
crtp_interface__srv__CrtpPacketSend_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__srv__CrtpPacketSend_Request__Sequence * array = (crtp_interface__srv__CrtpPacketSend_Request__Sequence *)allocator.allocate(sizeof(crtp_interface__srv__CrtpPacketSend_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crtp_interface__srv__CrtpPacketSend_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crtp_interface__srv__CrtpPacketSend_Request__Sequence__destroy(crtp_interface__srv__CrtpPacketSend_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crtp_interface__srv__CrtpPacketSend_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crtp_interface__srv__CrtpPacketSend_Request__Sequence__are_equal(const crtp_interface__srv__CrtpPacketSend_Request__Sequence * lhs, const crtp_interface__srv__CrtpPacketSend_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crtp_interface__srv__CrtpPacketSend_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crtp_interface__srv__CrtpPacketSend_Request__Sequence__copy(
  const crtp_interface__srv__CrtpPacketSend_Request__Sequence * input,
  crtp_interface__srv__CrtpPacketSend_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crtp_interface__srv__CrtpPacketSend_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crtp_interface__srv__CrtpPacketSend_Request * data =
      (crtp_interface__srv__CrtpPacketSend_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crtp_interface__srv__CrtpPacketSend_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crtp_interface__srv__CrtpPacketSend_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crtp_interface__srv__CrtpPacketSend_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
crtp_interface__srv__CrtpPacketSend_Response__init(crtp_interface__srv__CrtpPacketSend_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
crtp_interface__srv__CrtpPacketSend_Response__fini(crtp_interface__srv__CrtpPacketSend_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
crtp_interface__srv__CrtpPacketSend_Response__are_equal(const crtp_interface__srv__CrtpPacketSend_Response * lhs, const crtp_interface__srv__CrtpPacketSend_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
crtp_interface__srv__CrtpPacketSend_Response__copy(
  const crtp_interface__srv__CrtpPacketSend_Response * input,
  crtp_interface__srv__CrtpPacketSend_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

crtp_interface__srv__CrtpPacketSend_Response *
crtp_interface__srv__CrtpPacketSend_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__srv__CrtpPacketSend_Response * msg = (crtp_interface__srv__CrtpPacketSend_Response *)allocator.allocate(sizeof(crtp_interface__srv__CrtpPacketSend_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crtp_interface__srv__CrtpPacketSend_Response));
  bool success = crtp_interface__srv__CrtpPacketSend_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crtp_interface__srv__CrtpPacketSend_Response__destroy(crtp_interface__srv__CrtpPacketSend_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crtp_interface__srv__CrtpPacketSend_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crtp_interface__srv__CrtpPacketSend_Response__Sequence__init(crtp_interface__srv__CrtpPacketSend_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__srv__CrtpPacketSend_Response * data = NULL;

  if (size) {
    data = (crtp_interface__srv__CrtpPacketSend_Response *)allocator.zero_allocate(size, sizeof(crtp_interface__srv__CrtpPacketSend_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crtp_interface__srv__CrtpPacketSend_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crtp_interface__srv__CrtpPacketSend_Response__fini(&data[i - 1]);
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
crtp_interface__srv__CrtpPacketSend_Response__Sequence__fini(crtp_interface__srv__CrtpPacketSend_Response__Sequence * array)
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
      crtp_interface__srv__CrtpPacketSend_Response__fini(&array->data[i]);
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

crtp_interface__srv__CrtpPacketSend_Response__Sequence *
crtp_interface__srv__CrtpPacketSend_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crtp_interface__srv__CrtpPacketSend_Response__Sequence * array = (crtp_interface__srv__CrtpPacketSend_Response__Sequence *)allocator.allocate(sizeof(crtp_interface__srv__CrtpPacketSend_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crtp_interface__srv__CrtpPacketSend_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crtp_interface__srv__CrtpPacketSend_Response__Sequence__destroy(crtp_interface__srv__CrtpPacketSend_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crtp_interface__srv__CrtpPacketSend_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crtp_interface__srv__CrtpPacketSend_Response__Sequence__are_equal(const crtp_interface__srv__CrtpPacketSend_Response__Sequence * lhs, const crtp_interface__srv__CrtpPacketSend_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crtp_interface__srv__CrtpPacketSend_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crtp_interface__srv__CrtpPacketSend_Response__Sequence__copy(
  const crtp_interface__srv__CrtpPacketSend_Response__Sequence * input,
  crtp_interface__srv__CrtpPacketSend_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crtp_interface__srv__CrtpPacketSend_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crtp_interface__srv__CrtpPacketSend_Response * data =
      (crtp_interface__srv__CrtpPacketSend_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crtp_interface__srv__CrtpPacketSend_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crtp_interface__srv__CrtpPacketSend_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crtp_interface__srv__CrtpPacketSend_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
