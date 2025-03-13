// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robo24_interfaces:srv/Claw.idl
// generated code does not contain a copyright notice
#include "robo24_interfaces/srv/detail/claw__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
robo24_interfaces__srv__Claw_Request__init(robo24_interfaces__srv__Claw_Request * msg)
{
  if (!msg) {
    return false;
  }
  // pct
  // msec
  return true;
}

void
robo24_interfaces__srv__Claw_Request__fini(robo24_interfaces__srv__Claw_Request * msg)
{
  if (!msg) {
    return;
  }
  // pct
  // msec
}

bool
robo24_interfaces__srv__Claw_Request__are_equal(const robo24_interfaces__srv__Claw_Request * lhs, const robo24_interfaces__srv__Claw_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pct
  if (lhs->pct != rhs->pct) {
    return false;
  }
  // msec
  if (lhs->msec != rhs->msec) {
    return false;
  }
  return true;
}

bool
robo24_interfaces__srv__Claw_Request__copy(
  const robo24_interfaces__srv__Claw_Request * input,
  robo24_interfaces__srv__Claw_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // pct
  output->pct = input->pct;
  // msec
  output->msec = input->msec;
  return true;
}

robo24_interfaces__srv__Claw_Request *
robo24_interfaces__srv__Claw_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robo24_interfaces__srv__Claw_Request * msg = (robo24_interfaces__srv__Claw_Request *)allocator.allocate(sizeof(robo24_interfaces__srv__Claw_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robo24_interfaces__srv__Claw_Request));
  bool success = robo24_interfaces__srv__Claw_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robo24_interfaces__srv__Claw_Request__destroy(robo24_interfaces__srv__Claw_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robo24_interfaces__srv__Claw_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robo24_interfaces__srv__Claw_Request__Sequence__init(robo24_interfaces__srv__Claw_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robo24_interfaces__srv__Claw_Request * data = NULL;

  if (size) {
    data = (robo24_interfaces__srv__Claw_Request *)allocator.zero_allocate(size, sizeof(robo24_interfaces__srv__Claw_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robo24_interfaces__srv__Claw_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robo24_interfaces__srv__Claw_Request__fini(&data[i - 1]);
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
robo24_interfaces__srv__Claw_Request__Sequence__fini(robo24_interfaces__srv__Claw_Request__Sequence * array)
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
      robo24_interfaces__srv__Claw_Request__fini(&array->data[i]);
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

robo24_interfaces__srv__Claw_Request__Sequence *
robo24_interfaces__srv__Claw_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robo24_interfaces__srv__Claw_Request__Sequence * array = (robo24_interfaces__srv__Claw_Request__Sequence *)allocator.allocate(sizeof(robo24_interfaces__srv__Claw_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robo24_interfaces__srv__Claw_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robo24_interfaces__srv__Claw_Request__Sequence__destroy(robo24_interfaces__srv__Claw_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robo24_interfaces__srv__Claw_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robo24_interfaces__srv__Claw_Request__Sequence__are_equal(const robo24_interfaces__srv__Claw_Request__Sequence * lhs, const robo24_interfaces__srv__Claw_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robo24_interfaces__srv__Claw_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robo24_interfaces__srv__Claw_Request__Sequence__copy(
  const robo24_interfaces__srv__Claw_Request__Sequence * input,
  robo24_interfaces__srv__Claw_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robo24_interfaces__srv__Claw_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robo24_interfaces__srv__Claw_Request * data =
      (robo24_interfaces__srv__Claw_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robo24_interfaces__srv__Claw_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robo24_interfaces__srv__Claw_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robo24_interfaces__srv__Claw_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `resp`
#include "rosidl_runtime_c/string_functions.h"

bool
robo24_interfaces__srv__Claw_Response__init(robo24_interfaces__srv__Claw_Response * msg)
{
  if (!msg) {
    return false;
  }
  // resp
  if (!rosidl_runtime_c__String__init(&msg->resp)) {
    robo24_interfaces__srv__Claw_Response__fini(msg);
    return false;
  }
  return true;
}

void
robo24_interfaces__srv__Claw_Response__fini(robo24_interfaces__srv__Claw_Response * msg)
{
  if (!msg) {
    return;
  }
  // resp
  rosidl_runtime_c__String__fini(&msg->resp);
}

bool
robo24_interfaces__srv__Claw_Response__are_equal(const robo24_interfaces__srv__Claw_Response * lhs, const robo24_interfaces__srv__Claw_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // resp
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->resp), &(rhs->resp)))
  {
    return false;
  }
  return true;
}

bool
robo24_interfaces__srv__Claw_Response__copy(
  const robo24_interfaces__srv__Claw_Response * input,
  robo24_interfaces__srv__Claw_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // resp
  if (!rosidl_runtime_c__String__copy(
      &(input->resp), &(output->resp)))
  {
    return false;
  }
  return true;
}

robo24_interfaces__srv__Claw_Response *
robo24_interfaces__srv__Claw_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robo24_interfaces__srv__Claw_Response * msg = (robo24_interfaces__srv__Claw_Response *)allocator.allocate(sizeof(robo24_interfaces__srv__Claw_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robo24_interfaces__srv__Claw_Response));
  bool success = robo24_interfaces__srv__Claw_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robo24_interfaces__srv__Claw_Response__destroy(robo24_interfaces__srv__Claw_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robo24_interfaces__srv__Claw_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robo24_interfaces__srv__Claw_Response__Sequence__init(robo24_interfaces__srv__Claw_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robo24_interfaces__srv__Claw_Response * data = NULL;

  if (size) {
    data = (robo24_interfaces__srv__Claw_Response *)allocator.zero_allocate(size, sizeof(robo24_interfaces__srv__Claw_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robo24_interfaces__srv__Claw_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robo24_interfaces__srv__Claw_Response__fini(&data[i - 1]);
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
robo24_interfaces__srv__Claw_Response__Sequence__fini(robo24_interfaces__srv__Claw_Response__Sequence * array)
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
      robo24_interfaces__srv__Claw_Response__fini(&array->data[i]);
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

robo24_interfaces__srv__Claw_Response__Sequence *
robo24_interfaces__srv__Claw_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robo24_interfaces__srv__Claw_Response__Sequence * array = (robo24_interfaces__srv__Claw_Response__Sequence *)allocator.allocate(sizeof(robo24_interfaces__srv__Claw_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robo24_interfaces__srv__Claw_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robo24_interfaces__srv__Claw_Response__Sequence__destroy(robo24_interfaces__srv__Claw_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robo24_interfaces__srv__Claw_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robo24_interfaces__srv__Claw_Response__Sequence__are_equal(const robo24_interfaces__srv__Claw_Response__Sequence * lhs, const robo24_interfaces__srv__Claw_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robo24_interfaces__srv__Claw_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robo24_interfaces__srv__Claw_Response__Sequence__copy(
  const robo24_interfaces__srv__Claw_Response__Sequence * input,
  robo24_interfaces__srv__Claw_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robo24_interfaces__srv__Claw_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robo24_interfaces__srv__Claw_Response * data =
      (robo24_interfaces__srv__Claw_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robo24_interfaces__srv__Claw_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robo24_interfaces__srv__Claw_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robo24_interfaces__srv__Claw_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
