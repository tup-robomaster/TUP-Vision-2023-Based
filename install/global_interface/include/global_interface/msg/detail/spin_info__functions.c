// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from global_interface:msg/SpinInfo.idl
// generated code does not contain a copyright notice
#include "global_interface/msg/detail/spin_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
global_interface__msg__SpinInfo__init(global_interface__msg__SpinInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    global_interface__msg__SpinInfo__fini(msg);
    return false;
  }
  // is_spinning
  return true;
}

void
global_interface__msg__SpinInfo__fini(global_interface__msg__SpinInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_spinning
}

bool
global_interface__msg__SpinInfo__are_equal(const global_interface__msg__SpinInfo * lhs, const global_interface__msg__SpinInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // is_spinning
  if (lhs->is_spinning != rhs->is_spinning) {
    return false;
  }
  return true;
}

bool
global_interface__msg__SpinInfo__copy(
  const global_interface__msg__SpinInfo * input,
  global_interface__msg__SpinInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // is_spinning
  output->is_spinning = input->is_spinning;
  return true;
}

global_interface__msg__SpinInfo *
global_interface__msg__SpinInfo__create()
{
  global_interface__msg__SpinInfo * msg = (global_interface__msg__SpinInfo *)malloc(sizeof(global_interface__msg__SpinInfo));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(global_interface__msg__SpinInfo));
  bool success = global_interface__msg__SpinInfo__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
global_interface__msg__SpinInfo__destroy(global_interface__msg__SpinInfo * msg)
{
  if (msg) {
    global_interface__msg__SpinInfo__fini(msg);
  }
  free(msg);
}


bool
global_interface__msg__SpinInfo__Sequence__init(global_interface__msg__SpinInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  global_interface__msg__SpinInfo * data = NULL;
  if (size) {
    data = (global_interface__msg__SpinInfo *)calloc(size, sizeof(global_interface__msg__SpinInfo));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = global_interface__msg__SpinInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        global_interface__msg__SpinInfo__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
global_interface__msg__SpinInfo__Sequence__fini(global_interface__msg__SpinInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      global_interface__msg__SpinInfo__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

global_interface__msg__SpinInfo__Sequence *
global_interface__msg__SpinInfo__Sequence__create(size_t size)
{
  global_interface__msg__SpinInfo__Sequence * array = (global_interface__msg__SpinInfo__Sequence *)malloc(sizeof(global_interface__msg__SpinInfo__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = global_interface__msg__SpinInfo__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
global_interface__msg__SpinInfo__Sequence__destroy(global_interface__msg__SpinInfo__Sequence * array)
{
  if (array) {
    global_interface__msg__SpinInfo__Sequence__fini(array);
  }
  free(array);
}

bool
global_interface__msg__SpinInfo__Sequence__are_equal(const global_interface__msg__SpinInfo__Sequence * lhs, const global_interface__msg__SpinInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!global_interface__msg__SpinInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
global_interface__msg__SpinInfo__Sequence__copy(
  const global_interface__msg__SpinInfo__Sequence * input,
  global_interface__msg__SpinInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(global_interface__msg__SpinInfo);
    global_interface__msg__SpinInfo * data =
      (global_interface__msg__SpinInfo *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!global_interface__msg__SpinInfo__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          global_interface__msg__SpinInfo__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!global_interface__msg__SpinInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
