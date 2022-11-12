// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from global_interface:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__IMU__STRUCT_H_
#define GLOBAL_INTERFACE__MSG__DETAIL__IMU__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'quat'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

// Struct defined in msg/Imu in the package global_interface.
typedef struct global_interface__msg__Imu
{
  uint8_t bullet_speed;
  geometry_msgs__msg__Quaternion quat;
  geometry_msgs__msg__Twist twist;
} global_interface__msg__Imu;

// Struct for a sequence of global_interface__msg__Imu.
typedef struct global_interface__msg__Imu__Sequence
{
  global_interface__msg__Imu * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} global_interface__msg__Imu__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__IMU__STRUCT_H_
