// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from global_interface:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__IMU__BUILDER_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__IMU__BUILDER_HPP_

#include "global_interface/msg/detail/imu__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace global_interface
{

namespace msg
{

namespace builder
{

class Init_Imu_twist
{
public:
  explicit Init_Imu_twist(::global_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  ::global_interface::msg::Imu twist(::global_interface::msg::Imu::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::global_interface::msg::Imu msg_;
};

class Init_Imu_quat
{
public:
  explicit Init_Imu_quat(::global_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_twist quat(::global_interface::msg::Imu::_quat_type arg)
  {
    msg_.quat = std::move(arg);
    return Init_Imu_twist(msg_);
  }

private:
  ::global_interface::msg::Imu msg_;
};

class Init_Imu_bullet_speed
{
public:
  Init_Imu_bullet_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Imu_quat bullet_speed(::global_interface::msg::Imu::_bullet_speed_type arg)
  {
    msg_.bullet_speed = std::move(arg);
    return Init_Imu_quat(msg_);
  }

private:
  ::global_interface::msg::Imu msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::global_interface::msg::Imu>()
{
  return global_interface::msg::builder::Init_Imu_bullet_speed();
}

}  // namespace global_interface

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__IMU__BUILDER_HPP_
