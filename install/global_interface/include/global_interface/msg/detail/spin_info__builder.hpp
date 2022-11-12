// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from global_interface:msg/SpinInfo.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__SPIN_INFO__BUILDER_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__SPIN_INFO__BUILDER_HPP_

#include "global_interface/msg/detail/spin_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace global_interface
{

namespace msg
{

namespace builder
{

class Init_SpinInfo_is_spinning
{
public:
  explicit Init_SpinInfo_is_spinning(::global_interface::msg::SpinInfo & msg)
  : msg_(msg)
  {}
  ::global_interface::msg::SpinInfo is_spinning(::global_interface::msg::SpinInfo::_is_spinning_type arg)
  {
    msg_.is_spinning = std::move(arg);
    return std::move(msg_);
  }

private:
  ::global_interface::msg::SpinInfo msg_;
};

class Init_SpinInfo_header
{
public:
  Init_SpinInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SpinInfo_is_spinning header(::global_interface::msg::SpinInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SpinInfo_is_spinning(msg_);
  }

private:
  ::global_interface::msg::SpinInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::global_interface::msg::SpinInfo>()
{
  return global_interface::msg::builder::Init_SpinInfo_header();
}

}  // namespace global_interface

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__SPIN_INFO__BUILDER_HPP_
