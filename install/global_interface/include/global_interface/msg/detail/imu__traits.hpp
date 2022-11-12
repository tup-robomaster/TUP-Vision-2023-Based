// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from global_interface:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__IMU__TRAITS_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__IMU__TRAITS_HPP_

#include "global_interface/msg/detail/imu__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'quat'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const global_interface::msg::Imu & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bullet_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bullet_speed: ";
    value_to_yaml(msg.bullet_speed, out);
    out << "\n";
  }

  // member: quat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quat:\n";
    to_yaml(msg.quat, out, indentation + 2);
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_yaml(msg.twist, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const global_interface::msg::Imu & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<global_interface::msg::Imu>()
{
  return "global_interface::msg::Imu";
}

template<>
inline const char * name<global_interface::msg::Imu>()
{
  return "global_interface/msg/Imu";
}

template<>
struct has_fixed_size<global_interface::msg::Imu>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<global_interface::msg::Imu>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<global_interface::msg::Imu>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__IMU__TRAITS_HPP_
