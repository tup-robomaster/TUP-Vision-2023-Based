// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from global_interface:msg/Armor.idl
// generated code does not contain a copyright notice
#include "global_interface/msg/detail/armor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `key`
#include "rosidl_runtime_c/string_functions.h"
// Member `center3d_cam`
// Member `center3d_world`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
global_interface__msg__Armor__init(global_interface__msg__Armor * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // color
  // area
  // conf
  // key
  if (!rosidl_runtime_c__String__init(&msg->key)) {
    global_interface__msg__Armor__fini(msg);
    return false;
  }
  // center3d_cam
  if (!geometry_msgs__msg__Point__init(&msg->center3d_cam)) {
    global_interface__msg__Armor__fini(msg);
    return false;
  }
  // center3d_world
  if (!geometry_msgs__msg__Point__init(&msg->center3d_world)) {
    global_interface__msg__Armor__fini(msg);
    return false;
  }
  // type
  // dead_buffer_cnt
  return true;
}

void
global_interface__msg__Armor__fini(global_interface__msg__Armor * msg)
{
  if (!msg) {
    return;
  }
  // id
  // color
  // area
  // conf
  // key
  rosidl_runtime_c__String__fini(&msg->key);
  // center3d_cam
  geometry_msgs__msg__Point__fini(&msg->center3d_cam);
  // center3d_world
  geometry_msgs__msg__Point__fini(&msg->center3d_world);
  // type
  // dead_buffer_cnt
}

bool
global_interface__msg__Armor__are_equal(const global_interface__msg__Armor * lhs, const global_interface__msg__Armor * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // color
  if (lhs->color != rhs->color) {
    return false;
  }
  // area
  if (lhs->area != rhs->area) {
    return false;
  }
  // conf
  if (lhs->conf != rhs->conf) {
    return false;
  }
  // key
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->key), &(rhs->key)))
  {
    return false;
  }
  // center3d_cam
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->center3d_cam), &(rhs->center3d_cam)))
  {
    return false;
  }
  // center3d_world
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->center3d_world), &(rhs->center3d_world)))
  {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // dead_buffer_cnt
  if (lhs->dead_buffer_cnt != rhs->dead_buffer_cnt) {
    return false;
  }
  return true;
}

bool
global_interface__msg__Armor__copy(
  const global_interface__msg__Armor * input,
  global_interface__msg__Armor * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // color
  output->color = input->color;
  // area
  output->area = input->area;
  // conf
  output->conf = input->conf;
  // key
  if (!rosidl_runtime_c__String__copy(
      &(input->key), &(output->key)))
  {
    return false;
  }
  // center3d_cam
  if (!geometry_msgs__msg__Point__copy(
      &(input->center3d_cam), &(output->center3d_cam)))
  {
    return false;
  }
  // center3d_world
  if (!geometry_msgs__msg__Point__copy(
      &(input->center3d_world), &(output->center3d_world)))
  {
    return false;
  }
  // type
  output->type = input->type;
  // dead_buffer_cnt
  output->dead_buffer_cnt = input->dead_buffer_cnt;
  return true;
}

global_interface__msg__Armor *
global_interface__msg__Armor__create()
{
  global_interface__msg__Armor * msg = (global_interface__msg__Armor *)malloc(sizeof(global_interface__msg__Armor));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(global_interface__msg__Armor));
  bool success = global_interface__msg__Armor__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
global_interface__msg__Armor__destroy(global_interface__msg__Armor * msg)
{
  if (msg) {
    global_interface__msg__Armor__fini(msg);
  }
  free(msg);
}


bool
global_interface__msg__Armor__Sequence__init(global_interface__msg__Armor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  global_interface__msg__Armor * data = NULL;
  if (size) {
    data = (global_interface__msg__Armor *)calloc(size, sizeof(global_interface__msg__Armor));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = global_interface__msg__Armor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        global_interface__msg__Armor__fini(&data[i - 1]);
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
global_interface__msg__Armor__Sequence__fini(global_interface__msg__Armor__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      global_interface__msg__Armor__fini(&array->data[i]);
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

global_interface__msg__Armor__Sequence *
global_interface__msg__Armor__Sequence__create(size_t size)
{
  global_interface__msg__Armor__Sequence * array = (global_interface__msg__Armor__Sequence *)malloc(sizeof(global_interface__msg__Armor__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = global_interface__msg__Armor__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
global_interface__msg__Armor__Sequence__destroy(global_interface__msg__Armor__Sequence * array)
{
  if (array) {
    global_interface__msg__Armor__Sequence__fini(array);
  }
  free(array);
}

bool
global_interface__msg__Armor__Sequence__are_equal(const global_interface__msg__Armor__Sequence * lhs, const global_interface__msg__Armor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!global_interface__msg__Armor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
global_interface__msg__Armor__Sequence__copy(
  const global_interface__msg__Armor__Sequence * input,
  global_interface__msg__Armor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(global_interface__msg__Armor);
    global_interface__msg__Armor * data =
      (global_interface__msg__Armor *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!global_interface__msg__Armor__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          global_interface__msg__Armor__fini(&data[i]);
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
    if (!global_interface__msg__Armor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
