// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from global_interface:msg/Imu.idl
// generated code does not contain a copyright notice
#include "global_interface/msg/detail/imu__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `quat`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
global_interface__msg__Imu__init(global_interface__msg__Imu * msg)
{
  if (!msg) {
    return false;
  }
  // bullet_speed
  // quat
  if (!geometry_msgs__msg__Quaternion__init(&msg->quat)) {
    global_interface__msg__Imu__fini(msg);
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    global_interface__msg__Imu__fini(msg);
    return false;
  }
  return true;
}

void
global_interface__msg__Imu__fini(global_interface__msg__Imu * msg)
{
  if (!msg) {
    return;
  }
  // bullet_speed
  // quat
  geometry_msgs__msg__Quaternion__fini(&msg->quat);
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
}

bool
global_interface__msg__Imu__are_equal(const global_interface__msg__Imu * lhs, const global_interface__msg__Imu * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bullet_speed
  if (lhs->bullet_speed != rhs->bullet_speed) {
    return false;
  }
  // quat
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->quat), &(rhs->quat)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  return true;
}

bool
global_interface__msg__Imu__copy(
  const global_interface__msg__Imu * input,
  global_interface__msg__Imu * output)
{
  if (!input || !output) {
    return false;
  }
  // bullet_speed
  output->bullet_speed = input->bullet_speed;
  // quat
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->quat), &(output->quat)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  return true;
}

global_interface__msg__Imu *
global_interface__msg__Imu__create()
{
  global_interface__msg__Imu * msg = (global_interface__msg__Imu *)malloc(sizeof(global_interface__msg__Imu));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(global_interface__msg__Imu));
  bool success = global_interface__msg__Imu__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
global_interface__msg__Imu__destroy(global_interface__msg__Imu * msg)
{
  if (msg) {
    global_interface__msg__Imu__fini(msg);
  }
  free(msg);
}


bool
global_interface__msg__Imu__Sequence__init(global_interface__msg__Imu__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  global_interface__msg__Imu * data = NULL;
  if (size) {
    data = (global_interface__msg__Imu *)calloc(size, sizeof(global_interface__msg__Imu));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = global_interface__msg__Imu__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        global_interface__msg__Imu__fini(&data[i - 1]);
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
global_interface__msg__Imu__Sequence__fini(global_interface__msg__Imu__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      global_interface__msg__Imu__fini(&array->data[i]);
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

global_interface__msg__Imu__Sequence *
global_interface__msg__Imu__Sequence__create(size_t size)
{
  global_interface__msg__Imu__Sequence * array = (global_interface__msg__Imu__Sequence *)malloc(sizeof(global_interface__msg__Imu__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = global_interface__msg__Imu__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
global_interface__msg__Imu__Sequence__destroy(global_interface__msg__Imu__Sequence * array)
{
  if (array) {
    global_interface__msg__Imu__Sequence__fini(array);
  }
  free(array);
}

bool
global_interface__msg__Imu__Sequence__are_equal(const global_interface__msg__Imu__Sequence * lhs, const global_interface__msg__Imu__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!global_interface__msg__Imu__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
global_interface__msg__Imu__Sequence__copy(
  const global_interface__msg__Imu__Sequence * input,
  global_interface__msg__Imu__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(global_interface__msg__Imu);
    global_interface__msg__Imu * data =
      (global_interface__msg__Imu *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!global_interface__msg__Imu__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          global_interface__msg__Imu__fini(&data[i]);
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
    if (!global_interface__msg__Imu__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
