// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from global_interface:msg/Target.idl
// generated code does not contain a copyright notice
#include "global_interface/msg/detail/target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `aiming_point`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `rmat_imu`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
global_interface__msg__Target__init(global_interface__msg__Target * msg)
{
  if (!msg) {
    return false;
  }
  // aiming_point
  if (!geometry_msgs__msg__Point__init(&msg->aiming_point)) {
    global_interface__msg__Target__fini(msg);
    return false;
  }
  // timestamp
  // rmat_imu
  if (!geometry_msgs__msg__Vector3__init(&msg->rmat_imu)) {
    global_interface__msg__Target__fini(msg);
    return false;
  }
  return true;
}

void
global_interface__msg__Target__fini(global_interface__msg__Target * msg)
{
  if (!msg) {
    return;
  }
  // aiming_point
  geometry_msgs__msg__Point__fini(&msg->aiming_point);
  // timestamp
  // rmat_imu
  geometry_msgs__msg__Vector3__fini(&msg->rmat_imu);
}

bool
global_interface__msg__Target__are_equal(const global_interface__msg__Target * lhs, const global_interface__msg__Target * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // aiming_point
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->aiming_point), &(rhs->aiming_point)))
  {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // rmat_imu
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->rmat_imu), &(rhs->rmat_imu)))
  {
    return false;
  }
  return true;
}

bool
global_interface__msg__Target__copy(
  const global_interface__msg__Target * input,
  global_interface__msg__Target * output)
{
  if (!input || !output) {
    return false;
  }
  // aiming_point
  if (!geometry_msgs__msg__Point__copy(
      &(input->aiming_point), &(output->aiming_point)))
  {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // rmat_imu
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->rmat_imu), &(output->rmat_imu)))
  {
    return false;
  }
  return true;
}

global_interface__msg__Target *
global_interface__msg__Target__create()
{
  global_interface__msg__Target * msg = (global_interface__msg__Target *)malloc(sizeof(global_interface__msg__Target));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(global_interface__msg__Target));
  bool success = global_interface__msg__Target__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
global_interface__msg__Target__destroy(global_interface__msg__Target * msg)
{
  if (msg) {
    global_interface__msg__Target__fini(msg);
  }
  free(msg);
}


bool
global_interface__msg__Target__Sequence__init(global_interface__msg__Target__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  global_interface__msg__Target * data = NULL;
  if (size) {
    data = (global_interface__msg__Target *)calloc(size, sizeof(global_interface__msg__Target));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = global_interface__msg__Target__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        global_interface__msg__Target__fini(&data[i - 1]);
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
global_interface__msg__Target__Sequence__fini(global_interface__msg__Target__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      global_interface__msg__Target__fini(&array->data[i]);
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

global_interface__msg__Target__Sequence *
global_interface__msg__Target__Sequence__create(size_t size)
{
  global_interface__msg__Target__Sequence * array = (global_interface__msg__Target__Sequence *)malloc(sizeof(global_interface__msg__Target__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = global_interface__msg__Target__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
global_interface__msg__Target__Sequence__destroy(global_interface__msg__Target__Sequence * array)
{
  if (array) {
    global_interface__msg__Target__Sequence__fini(array);
  }
  free(array);
}

bool
global_interface__msg__Target__Sequence__are_equal(const global_interface__msg__Target__Sequence * lhs, const global_interface__msg__Target__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!global_interface__msg__Target__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
global_interface__msg__Target__Sequence__copy(
  const global_interface__msg__Target__Sequence * input,
  global_interface__msg__Target__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(global_interface__msg__Target);
    global_interface__msg__Target * data =
      (global_interface__msg__Target *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!global_interface__msg__Target__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          global_interface__msg__Target__fini(&data[i]);
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
    if (!global_interface__msg__Target__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
