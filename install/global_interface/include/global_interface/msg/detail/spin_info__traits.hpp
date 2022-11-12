// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from global_interface:msg/SpinInfo.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__SPIN_INFO__TRAITS_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__SPIN_INFO__TRAITS_HPP_

#include "global_interface/msg/detail/spin_info__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const global_interface::msg::SpinInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
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

inline std::string to_yaml(const global_interface::msg::SpinInfo & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<global_interface::msg::SpinInfo>()
{
  return "global_interface::msg::SpinInfo";
}

template<>
inline const char * name<global_interface::msg::SpinInfo>()
{
  return "global_interface/msg/SpinInfo";
}

template<>
struct has_fixed_size<global_interface::msg::SpinInfo>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<global_interface::msg::SpinInfo>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<global_interface::msg::SpinInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__SPIN_INFO__TRAITS_HPP_
