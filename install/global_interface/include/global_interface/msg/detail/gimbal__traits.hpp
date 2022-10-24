// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from global_interface:msg/Gimbal.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__TRAITS_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__TRAITS_HPP_

#include "global_interface/msg/detail/gimbal__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const global_interface::msg::Gimbal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: is_switched
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_switched: ";
    value_to_yaml(msg.is_switched, out);
    out << "\n";
  }

  // member: is_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_target: ";
    value_to_yaml(msg.is_target, out);
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

  // member: is_middle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_middle: ";
    value_to_yaml(msg.is_middle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const global_interface::msg::Gimbal & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<global_interface::msg::Gimbal>()
{
  return "global_interface::msg::Gimbal";
}

template<>
inline const char * name<global_interface::msg::Gimbal>()
{
  return "global_interface/msg/Gimbal";
}

template<>
struct has_fixed_size<global_interface::msg::Gimbal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<global_interface::msg::Gimbal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<global_interface::msg::Gimbal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__TRAITS_HPP_
