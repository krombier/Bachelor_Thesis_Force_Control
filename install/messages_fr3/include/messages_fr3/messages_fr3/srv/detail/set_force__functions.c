// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages_fr3:srv/SetForce.idl
// generated code does not contain a copyright notice
#include "messages_fr3/srv/detail/set_force__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
messages_fr3__srv__SetForce_Request__init(messages_fr3__srv__SetForce_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x_force
  // y_force
  // z_force
  // x_torque
  // y_torque
  // z_torque
  // frame
  return true;
}

void
messages_fr3__srv__SetForce_Request__fini(messages_fr3__srv__SetForce_Request * msg)
{
  if (!msg) {
    return;
  }
  // x_force
  // y_force
  // z_force
  // x_torque
  // y_torque
  // z_torque
  // frame
}

bool
messages_fr3__srv__SetForce_Request__are_equal(const messages_fr3__srv__SetForce_Request * lhs, const messages_fr3__srv__SetForce_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_force
  if (lhs->x_force != rhs->x_force) {
    return false;
  }
  // y_force
  if (lhs->y_force != rhs->y_force) {
    return false;
  }
  // z_force
  if (lhs->z_force != rhs->z_force) {
    return false;
  }
  // x_torque
  if (lhs->x_torque != rhs->x_torque) {
    return false;
  }
  // y_torque
  if (lhs->y_torque != rhs->y_torque) {
    return false;
  }
  // z_torque
  if (lhs->z_torque != rhs->z_torque) {
    return false;
  }
  // frame
  if (lhs->frame != rhs->frame) {
    return false;
  }
  return true;
}

bool
messages_fr3__srv__SetForce_Request__copy(
  const messages_fr3__srv__SetForce_Request * input,
  messages_fr3__srv__SetForce_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x_force
  output->x_force = input->x_force;
  // y_force
  output->y_force = input->y_force;
  // z_force
  output->z_force = input->z_force;
  // x_torque
  output->x_torque = input->x_torque;
  // y_torque
  output->y_torque = input->y_torque;
  // z_torque
  output->z_torque = input->z_torque;
  // frame
  output->frame = input->frame;
  return true;
}

messages_fr3__srv__SetForce_Request *
messages_fr3__srv__SetForce_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages_fr3__srv__SetForce_Request * msg = (messages_fr3__srv__SetForce_Request *)allocator.allocate(sizeof(messages_fr3__srv__SetForce_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages_fr3__srv__SetForce_Request));
  bool success = messages_fr3__srv__SetForce_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
messages_fr3__srv__SetForce_Request__destroy(messages_fr3__srv__SetForce_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    messages_fr3__srv__SetForce_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
messages_fr3__srv__SetForce_Request__Sequence__init(messages_fr3__srv__SetForce_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages_fr3__srv__SetForce_Request * data = NULL;

  if (size) {
    data = (messages_fr3__srv__SetForce_Request *)allocator.zero_allocate(size, sizeof(messages_fr3__srv__SetForce_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages_fr3__srv__SetForce_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages_fr3__srv__SetForce_Request__fini(&data[i - 1]);
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
messages_fr3__srv__SetForce_Request__Sequence__fini(messages_fr3__srv__SetForce_Request__Sequence * array)
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
      messages_fr3__srv__SetForce_Request__fini(&array->data[i]);
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

messages_fr3__srv__SetForce_Request__Sequence *
messages_fr3__srv__SetForce_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages_fr3__srv__SetForce_Request__Sequence * array = (messages_fr3__srv__SetForce_Request__Sequence *)allocator.allocate(sizeof(messages_fr3__srv__SetForce_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = messages_fr3__srv__SetForce_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
messages_fr3__srv__SetForce_Request__Sequence__destroy(messages_fr3__srv__SetForce_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    messages_fr3__srv__SetForce_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
messages_fr3__srv__SetForce_Request__Sequence__are_equal(const messages_fr3__srv__SetForce_Request__Sequence * lhs, const messages_fr3__srv__SetForce_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages_fr3__srv__SetForce_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages_fr3__srv__SetForce_Request__Sequence__copy(
  const messages_fr3__srv__SetForce_Request__Sequence * input,
  messages_fr3__srv__SetForce_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages_fr3__srv__SetForce_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    messages_fr3__srv__SetForce_Request * data =
      (messages_fr3__srv__SetForce_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages_fr3__srv__SetForce_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          messages_fr3__srv__SetForce_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!messages_fr3__srv__SetForce_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
messages_fr3__srv__SetForce_Response__init(messages_fr3__srv__SetForce_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
messages_fr3__srv__SetForce_Response__fini(messages_fr3__srv__SetForce_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
messages_fr3__srv__SetForce_Response__are_equal(const messages_fr3__srv__SetForce_Response * lhs, const messages_fr3__srv__SetForce_Response * rhs)
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
messages_fr3__srv__SetForce_Response__copy(
  const messages_fr3__srv__SetForce_Response * input,
  messages_fr3__srv__SetForce_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

messages_fr3__srv__SetForce_Response *
messages_fr3__srv__SetForce_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages_fr3__srv__SetForce_Response * msg = (messages_fr3__srv__SetForce_Response *)allocator.allocate(sizeof(messages_fr3__srv__SetForce_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages_fr3__srv__SetForce_Response));
  bool success = messages_fr3__srv__SetForce_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
messages_fr3__srv__SetForce_Response__destroy(messages_fr3__srv__SetForce_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    messages_fr3__srv__SetForce_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
messages_fr3__srv__SetForce_Response__Sequence__init(messages_fr3__srv__SetForce_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages_fr3__srv__SetForce_Response * data = NULL;

  if (size) {
    data = (messages_fr3__srv__SetForce_Response *)allocator.zero_allocate(size, sizeof(messages_fr3__srv__SetForce_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages_fr3__srv__SetForce_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages_fr3__srv__SetForce_Response__fini(&data[i - 1]);
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
messages_fr3__srv__SetForce_Response__Sequence__fini(messages_fr3__srv__SetForce_Response__Sequence * array)
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
      messages_fr3__srv__SetForce_Response__fini(&array->data[i]);
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

messages_fr3__srv__SetForce_Response__Sequence *
messages_fr3__srv__SetForce_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages_fr3__srv__SetForce_Response__Sequence * array = (messages_fr3__srv__SetForce_Response__Sequence *)allocator.allocate(sizeof(messages_fr3__srv__SetForce_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = messages_fr3__srv__SetForce_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
messages_fr3__srv__SetForce_Response__Sequence__destroy(messages_fr3__srv__SetForce_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    messages_fr3__srv__SetForce_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
messages_fr3__srv__SetForce_Response__Sequence__are_equal(const messages_fr3__srv__SetForce_Response__Sequence * lhs, const messages_fr3__srv__SetForce_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages_fr3__srv__SetForce_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages_fr3__srv__SetForce_Response__Sequence__copy(
  const messages_fr3__srv__SetForce_Response__Sequence * input,
  messages_fr3__srv__SetForce_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages_fr3__srv__SetForce_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    messages_fr3__srv__SetForce_Response * data =
      (messages_fr3__srv__SetForce_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages_fr3__srv__SetForce_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          messages_fr3__srv__SetForce_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!messages_fr3__srv__SetForce_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
