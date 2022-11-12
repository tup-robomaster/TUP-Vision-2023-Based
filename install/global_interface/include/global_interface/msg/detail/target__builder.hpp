// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from global_interface:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__TARGET__BUILDER_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__TARGET__BUILDER_HPP_

#include "global_interface/msg/detail/target__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace global_interface
{

namespace msg
{

namespace builder
{

class Init_Target_is_spinning
{
public:
  explicit Init_Target_is_spinning(::global_interface::msg::Target & msg)
  : msg_(msg)
  {}
  ::global_interface::msg::Target is_spinning(::global_interface::msg::Target::_is_spinning_type arg)
  {
    msg_.is_spinning = std::move(arg);
    return std::move(msg_);
  }

private:
  ::global_interface::msg::Target msg_;
};

class Init_Target_target_switched
{
public:
  explicit Init_Target_target_switched(::global_interface::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_is_spinning target_switched(::global_interface::msg::Target::_target_switched_type arg)
  {
    msg_.target_switched = std::move(arg);
    return Init_Target_is_spinning(msg_);
  }

private:
  ::global_interface::msg::Target msg_;
};

class Init_Target_rmat_imu
{
public:
  explicit Init_Target_rmat_imu(::global_interface::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_target_switched rmat_imu(::global_interface::msg::Target::_rmat_imu_type arg)
  {
    msg_.rmat_imu = std::move(arg);
    return Init_Target_target_switched(msg_);
  }

private:
  ::global_interface::msg::Target msg_;
};

class Init_Target_timestamp
{
public:
  explicit Init_Target_timestamp(::global_interface::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_rmat_imu timestamp(::global_interface::msg::Target::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Target_rmat_imu(msg_);
  }

private:
  ::global_interface::msg::Target msg_;
};

class Init_Target_aiming_point
{
public:
  Init_Target_aiming_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_timestamp aiming_point(::global_interface::msg::Target::_aiming_point_type arg)
  {
    msg_.aiming_point = std::move(arg);
    return Init_Target_timestamp(msg_);
  }

private:
  ::global_interface::msg::Target msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::global_interface::msg::Target>()
{
  return global_interface::msg::builder::Init_Target_aiming_point();
}

}  // namespace global_interface

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__TARGET__BUILDER_HPP_
