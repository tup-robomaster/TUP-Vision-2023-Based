// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from global_interface:msg/Armor.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__STRUCT_H_
#define GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'key'
#include "rosidl_runtime_c/string.h"
// Member 'center3d_cam'
// Member 'center3d_world'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in msg/Armor in the package global_interface.
typedef struct global_interface__msg__Armor
{
  uint8_t id;
  uint8_t color;
  int32_t area;
  float conf;
  rosidl_runtime_c__String key;
  geometry_msgs__msg__Point center3d_cam;
  geometry_msgs__msg__Point center3d_world;
  uint8_t type;
  uint8_t dead_buffer_cnt;
} global_interface__msg__Armor;

// Struct for a sequence of global_interface__msg__Armor.
typedef struct global_interface__msg__Armor__Sequence
{
  global_interface__msg__Armor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} global_interface__msg__Armor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__STRUCT_H_
