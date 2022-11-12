// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from global_interface:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__TARGET__TRAITS_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__TARGET__TRAITS_HPP_

#include "global_interface/msg/detail/target__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'aiming_point'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'rmat_imu'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const global_interface::msg::Target & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: aiming_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aiming_point:\n";
    to_yaml(msg.aiming_point, out, indentation + 2);
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: rmat_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rmat_imu:\n";
    to_yaml(msg.rmat_imu, out, indentation + 2);
  }

  // member: target_switched
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_switched: ";
    value_to_yaml(msg.target_switched, out);
    out << "\n";
  }

  // member: is_spinning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_spinning: ";
    value_to_yaml(msg.is_spinning, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const global_interface::msg::Target & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<global_interface::msg::Target>()
{
  return "global_interface::msg::Target";
}

template<>
inline const char * name<global_interface::msg::Target>()
{
  return "global_interface/msg/Target";
}

template<>
struct has_fixed_size<global_interface::msg::Target>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<global_interface::msg::Target>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<global_interface::msg::Target>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__TARGET__TRAITS_HPP_
