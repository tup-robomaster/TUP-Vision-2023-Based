// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from global_interface:msg/Armor.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__STRUCT_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'center3d_cam'
// Member 'center3d_world'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__global_interface__msg__Armor __attribute__((deprecated))
#else
# define DEPRECATED__global_interface__msg__Armor __declspec(deprecated)
#endif

namespace global_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Armor_
{
  using Type = Armor_<ContainerAllocator>;

  explicit Armor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : center3d_cam(_init),
    center3d_world(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->color = 0;
      this->area = 0l;
      this->conf = 0.0f;
      this->key = "";
      this->type = 0;
      this->dead_buffer_cnt = 0;
    }
  }

  explicit Armor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : key(_alloc),
    center3d_cam(_alloc, _init),
    center3d_world(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->color = 0;
      this->area = 0l;
      this->conf = 0.0f;
      this->key = "";
      this->type = 0;
      this->dead_buffer_cnt = 0;
    }
  }

  // field types and members
  using _id_type =
    uint8_t;
  _id_type id;
  using _color_type =
    uint8_t;
  _color_type color;
  using _area_type =
    int32_t;
  _area_type area;
  using _conf_type =
    float;
  _conf_type conf;
  using _key_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _key_type key;
  using _center3d_cam_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _center3d_cam_type center3d_cam;
  using _center3d_world_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _center3d_world_type center3d_world;
  using _type_type =
    uint8_t;
  _type_type type;
  using _dead_buffer_cnt_type =
    uint8_t;
  _dead_buffer_cnt_type dead_buffer_cnt;

  // setters for named parameter idiom
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__color(
    const uint8_t & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__area(
    const int32_t & _arg)
  {
    this->area = _arg;
    return *this;
  }
  Type & set__conf(
    const float & _arg)
  {
    this->conf = _arg;
    return *this;
  }
  Type & set__key(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->key = _arg;
    return *this;
  }
  Type & set__center3d_cam(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->center3d_cam = _arg;
    return *this;
  }
  Type & set__center3d_world(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->center3d_world = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__dead_buffer_cnt(
    const uint8_t & _arg)
  {
    this->dead_buffer_cnt = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    global_interface::msg::Armor_<ContainerAllocator> *;
  using ConstRawPtr =
    const global_interface::msg::Armor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<global_interface::msg::Armor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<global_interface::msg::Armor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      global_interface::msg::Armor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<global_interface::msg::Armor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      global_interface::msg::Armor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<global_interface::msg::Armor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<global_interface::msg::Armor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<global_interface::msg::Armor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__global_interface__msg__Armor
    std::shared_ptr<global_interface::msg::Armor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__global_interface__msg__Armor
    std::shared_ptr<global_interface::msg::Armor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Armor_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    if (this->area != other.area) {
      return false;
    }
    if (this->conf != other.conf) {
      return false;
    }
    if (this->key != other.key) {
      return false;
    }
    if (this->center3d_cam != other.center3d_cam) {
      return false;
    }
    if (this->center3d_world != other.center3d_world) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->dead_buffer_cnt != other.dead_buffer_cnt) {
      return false;
    }
    return true;
  }
  bool operator!=(const Armor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Armor_

// alias to use template instance with default allocator
using Armor =
  global_interface::msg::Armor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace global_interface

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__ARMOR__STRUCT_HPP_
