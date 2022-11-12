// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from global_interface:msg/Armor.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__TRAITS_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__TRAITS_HPP_

#include "global_interface/msg/detail/armor__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'center3d_cam'
// Member 'center3d_world'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const global_interface::msg::Armor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    value_to_yaml(msg.color, out);
    out << "\n";
  }

  // member: area
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "area: ";
    value_to_yaml(msg.area, out);
    out << "\n";
  }

  // member: conf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "conf: ";
    value_to_yaml(msg.conf, out);
    out << "\n";
  }

  // member: key
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key: ";
    value_to_yaml(msg.key, out);
    out << "\n";
  }

  // member: center3d_cam
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center3d_cam:\n";
    to_yaml(msg.center3d_cam, out, indentation + 2);
  }

  // member: center3d_world
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center3d_world:\n";
    to_yaml(msg.center3d_world, out, indentation + 2);
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: dead_buffer_cnt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dead_buffer_cnt: ";
    value_to_yaml(msg.dead_buffer_cnt, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const global_interface::msg::Armor & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<global_interface::msg::Armor>()
{
  return "global_interface::msg::Armor";
}

template<>
inline const char * name<global_interface::msg::Armor>()
{
  return "global_interface/msg/Armor";
}

template<>
struct has_fixed_size<global_interface::msg::Armor>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<global_interface::msg::Armor>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<global_interface::msg::Armor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__TRAITS_HPP_
