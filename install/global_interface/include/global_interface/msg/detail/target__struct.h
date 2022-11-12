// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from global_interface:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__TARGET__STRUCT_H_
#define GLOBAL_INTERFACE__MSG__DETAIL__TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'aiming_point'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'rmat_imu'
#include "geometry_msgs/msg/detail/vector3__struct.h"

// Struct defined in msg/Target in the package global_interface.
typedef struct global_interface__msg__Target
{
  geometry_msgs__msg__Point aiming_point;
  double timestamp;
  geometry_msgs__msg__Vector3 rmat_imu;
  bool target_switched;
  bool is_spinning;
} global_interface__msg__Target;

// Struct for a sequence of global_interface__msg__Target.
typedef struct global_interface__msg__Target__Sequence
{
  global_interface__msg__Target * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} global_interface__msg__Target__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__TARGET__STRUCT_H_
