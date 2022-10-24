// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from global_interface:msg/Gimbal.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__BUILDER_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__BUILDER_HPP_

#include "global_interface/msg/detail/gimbal__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace global_interface
{

namespace msg
{

namespace builder
{

class Init_Gimbal_is_middle
{
public:
  explicit Init_Gimbal_is_middle(::global_interface::msg::Gimbal & msg)
  : msg_(msg)
  {}
  ::global_interface::msg::Gimbal is_middle(::global_interface::msg::Gimbal::_is_middle_type arg)
  {
    msg_.is_middle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::global_interface::msg::Gimbal msg_;
};

class Init_Gimbal_is_spinning
{
public:
  explicit Init_Gimbal_is_spinning(::global_interface::msg::Gimbal & msg)
  : msg_(msg)
  {}
  Init_Gimbal_is_middle is_spinning(::global_interface::msg::Gimbal::_is_spinning_type arg)
  {
    msg_.is_spinning = std::move(arg);
    return Init_Gimbal_is_middle(msg_);
  }

private:
  ::global_interface::msg::Gimbal msg_;
};

class Init_Gimbal_is_target
{
public:
  explicit Init_Gimbal_is_target(::global_interface::msg::Gimbal & msg)
  : msg_(msg)
  {}
  Init_Gimbal_is_spinning is_target(::global_interface::msg::Gimbal::_is_target_type arg)
  {
    msg_.is_target = std::move(arg);
    return Init_Gimbal_is_spinning(msg_);
  }

private:
  ::global_interface::msg::Gimbal msg_;
};

class Init_Gimbal_is_switched
{
public:
  explicit Init_Gimbal_is_switched(::global_interface::msg::Gimbal & msg)
  : msg_(msg)
  {}
  Init_Gimbal_is_target is_switched(::global_interface::msg::Gimbal::_is_switched_type arg)
  {
    msg_.is_switched = std::move(arg);
    return Init_Gimbal_is_target(msg_);
  }

private:
  ::global_interface::msg::Gimbal msg_;
};

class Init_Gimbal_distance
{
public:
  explicit Init_Gimbal_distance(::global_interface::msg::Gimbal & msg)
  : msg_(msg)
  {}
  Init_Gimbal_is_switched distance(::global_interface::msg::Gimbal::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_Gimbal_is_switched(msg_);
  }

private:
  ::global_interface::msg::Gimbal msg_;
};

class Init_Gimbal_yaw
{
public:
  explicit Init_Gimbal_yaw(::global_interface::msg::Gimbal & msg)
  : msg_(msg)
  {}
  Init_Gimbal_distance yaw(::global_interface::msg::Gimbal::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Gimbal_distance(msg_);
  }

private:
  ::global_interface::msg::Gimbal msg_;
};

class Init_Gimbal_pitch
{
public:
  Init_Gimbal_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gimbal_yaw pitch(::global_interface::msg::Gimbal::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Gimbal_yaw(msg_);
  }

private:
  ::global_interface::msg::Gimbal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::global_interface::msg::Gimbal>()
{
  return global_interface::msg::builder::Init_Gimbal_pitch();
}

}  // namespace global_interface

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__BUILDER_HPP_
