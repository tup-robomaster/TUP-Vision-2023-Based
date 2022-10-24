// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from global_interface:msg/Gimbal.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__STRUCT_H_
#define GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Gimbal in the package global_interface.
typedef struct global_interface__msg__Gimbal
{
  double pitch;
  double yaw;
  double distance;
  bool is_switched;
  bool is_target;
  bool is_spinning;
  bool is_middle;
} global_interface__msg__Gimbal;

// Struct for a sequence of global_interface__msg__Gimbal.
typedef struct global_interface__msg__Gimbal__Sequence
{
  global_interface__msg__Gimbal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} global_interface__msg__Gimbal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__STRUCT_H_
