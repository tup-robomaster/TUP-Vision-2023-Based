// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from global_interface:msg/Gimbal.idl
// generated code does not contain a copyright notice
#include "global_interface/msg/detail/gimbal__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
global_interface__msg__Gimbal__init(global_interface__msg__Gimbal * msg)
{
  if (!msg) {
    return false;
  }
  // pitch
  // yaw
  // distance
  // is_switched
  // is_target
  // is_spinning
  // is_middle
  return true;
}

void
global_interface__msg__Gimbal__fini(global_interface__msg__Gimbal * msg)
{
  if (!msg) {
    return;
  }
  // pitch
  // yaw
  // distance
  // is_switched
  // is_target
  // is_spinning
  // is_middle
}

bool
global_interface__msg__Gimbal__are_equal(const global_interface__msg__Gimbal * lhs, const global_interface__msg__Gimbal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // is_switched
  if (lhs->is_switched != rhs->is_switched) {
    return false;
  }
  // is_target
  if (lhs->is_target != rhs->is_target) {
    return false;
  }
  // is_spinning
  if (lhs->is_spinning != rhs->is_spinning) {
    return false;
  }
  // is_middle
  if (lhs->is_middle != rhs->is_middle) {
    return false;
  }
  return true;
}

bool
global_interface__msg__Gimbal__copy(
  const global_interface__msg__Gimbal * input,
  global_interface__msg__Gimbal * output)
{
  if (!input || !output) {
    return false;
  }
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // distance
  output->distance = input->distance;
  // is_switched
  output->is_switched = input->is_switched;
  // is_target
  output->is_target = input->is_target;
  // is_spinning
  output->is_spinning = input->is_spinning;
  // is_middle
  output->is_middle = input->is_middle;
  return true;
}

global_interface__msg__Gimbal *
global_interface__msg__Gimbal__create()
{
  global_interface__msg__Gimbal * msg = (global_interface__msg__Gimbal *)malloc(sizeof(global_interface__msg__Gimbal));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(global_interface__msg__Gimbal));
  bool success = global_interface__msg__Gimbal__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
global_interface__msg__Gimbal__destroy(global_interface__msg__Gimbal * msg)
{
  if (msg) {
    global_interface__msg__Gimbal__fini(msg);
  }
  free(msg);
}


bool
global_interface__msg__Gimbal__Sequence__init(global_interface__msg__Gimbal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  global_interface__msg__Gimbal * data = NULL;
  if (size) {
    data = (global_interface__msg__Gimbal *)calloc(size, sizeof(global_interface__msg__Gimbal));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = global_interface__msg__Gimbal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        global_interface__msg__Gimbal__fini(&data[i - 1]);
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
global_interface__msg__Gimbal__Sequence__fini(global_interface__msg__Gimbal__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      global_interface__msg__Gimbal__fini(&array->data[i]);
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

global_interface__msg__Gimbal__Sequence *
global_interface__msg__Gimbal__Sequence__create(size_t size)
{
  global_interface__msg__Gimbal__Sequence * array = (global_interface__msg__Gimbal__Sequence *)malloc(sizeof(global_interface__msg__Gimbal__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = global_interface__msg__Gimbal__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
global_interface__msg__Gimbal__Sequence__destroy(global_interface__msg__Gimbal__Sequence * array)
{
  if (array) {
    global_interface__msg__Gimbal__Sequence__fini(array);
  }
  free(array);
}

bool
global_interface__msg__Gimbal__Sequence__are_equal(const global_interface__msg__Gimbal__Sequence * lhs, const global_interface__msg__Gimbal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!global_interface__msg__Gimbal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
global_interface__msg__Gimbal__Sequence__copy(
  const global_interface__msg__Gimbal__Sequence * input,
  global_interface__msg__Gimbal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(global_interface__msg__Gimbal);
    global_interface__msg__Gimbal * data =
      (global_interface__msg__Gimbal *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!global_interface__msg__Gimbal__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          global_interface__msg__Gimbal__fini(&data[i]);
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
    if (!global_interface__msg__Gimbal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
