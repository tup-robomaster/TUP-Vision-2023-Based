// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from global_interface:msg/Armor.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__BUILDER_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__BUILDER_HPP_

#include "global_interface/msg/detail/armor__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace global_interface
{

namespace msg
{

namespace builder
{

class Init_Armor_dead_buffer_cnt
{
public:
  explicit Init_Armor_dead_buffer_cnt(::global_interface::msg::Armor & msg)
  : msg_(msg)
  {}
  ::global_interface::msg::Armor dead_buffer_cnt(::global_interface::msg::Armor::_dead_buffer_cnt_type arg)
  {
    msg_.dead_buffer_cnt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::global_interface::msg::Armor msg_;
};

class Init_Armor_type
{
public:
  explicit Init_Armor_type(::global_interface::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_dead_buffer_cnt type(::global_interface::msg::Armor::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Armor_dead_buffer_cnt(msg_);
  }

private:
  ::global_interface::msg::Armor msg_;
};

class Init_Armor_center3d_world
{
public:
  explicit Init_Armor_center3d_world(::global_interface::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_type center3d_world(::global_interface::msg::Armor::_center3d_world_type arg)
  {
    msg_.center3d_world = std::move(arg);
    return Init_Armor_type(msg_);
  }

private:
  ::global_interface::msg::Armor msg_;
};

class Init_Armor_center3d_cam
{
public:
  explicit Init_Armor_center3d_cam(::global_interface::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_center3d_world center3d_cam(::global_interface::msg::Armor::_center3d_cam_type arg)
  {
    msg_.center3d_cam = std::move(arg);
    return Init_Armor_center3d_world(msg_);
  }

private:
  ::global_interface::msg::Armor msg_;
};

class Init_Armor_key
{
public:
  explicit Init_Armor_key(::global_interface::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_center3d_cam key(::global_interface::msg::Armor::_key_type arg)
  {
    msg_.key = std::move(arg);
    return Init_Armor_center3d_cam(msg_);
  }

private:
  ::global_interface::msg::Armor msg_;
};

class Init_Armor_conf
{
public:
  explicit Init_Armor_conf(::global_interface::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_key conf(::global_interface::msg::Armor::_conf_type arg)
  {
    msg_.conf = std::move(arg);
    return Init_Armor_key(msg_);
  }

private:
  ::global_interface::msg::Armor msg_;
};

class Init_Armor_area
{
public:
  explicit Init_Armor_area(::global_interface::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_conf area(::global_interface::msg::Armor::_area_type arg)
  {
    msg_.area = std::move(arg);
    return Init_Armor_conf(msg_);
  }

private:
  ::global_interface::msg::Armor msg_;
};

class Init_Armor_color
{
public:
  explicit Init_Armor_color(::global_interface::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_area color(::global_interface::msg::Armor::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_Armor_area(msg_);
  }

private:
  ::global_interface::msg::Armor msg_;
};

class Init_Armor_id
{
public:
  Init_Armor_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Armor_color id(::global_interface::msg::Armor::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Armor_color(msg_);
  }

private:
  ::global_interface::msg::Armor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::global_interface::msg::Armor>()
{
  return global_interface::msg::builder::Init_Armor_id();
}

}  // namespace global_interface

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__BUILDER_HPP_
