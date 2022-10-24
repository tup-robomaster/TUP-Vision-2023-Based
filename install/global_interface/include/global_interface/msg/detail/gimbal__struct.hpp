// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from global_interface:msg/Gimbal.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__STRUCT_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__global_interface__msg__Gimbal __attribute__((deprecated))
#else
# define DEPRECATED__global_interface__msg__Gimbal __declspec(deprecated)
#endif

namespace global_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gimbal_
{
  using Type = Gimbal_<ContainerAllocator>;

  explicit Gimbal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->distance = 0.0;
      this->is_switched = false;
      this->is_target = false;
      this->is_spinning = false;
      this->is_middle = false;
    }
  }

  explicit Gimbal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->distance = 0.0;
      this->is_switched = false;
      this->is_target = false;
      this->is_spinning = false;
      this->is_middle = false;
    }
  }

  // field types and members
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _distance_type =
    double;
  _distance_type distance;
  using _is_switched_type =
    bool;
  _is_switched_type is_switched;
  using _is_target_type =
    bool;
  _is_target_type is_target;
  using _is_spinning_type =
    bool;
  _is_spinning_type is_spinning;
  using _is_middle_type =
    bool;
  _is_middle_type is_middle;

  // setters for named parameter idiom
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__distance(
    const double & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__is_switched(
    const bool & _arg)
  {
    this->is_switched = _arg;
    return *this;
  }
  Type & set__is_target(
    const bool & _arg)
  {
    this->is_target = _arg;
    return *this;
  }
  Type & set__is_spinning(
    const bool & _arg)
  {
    this->is_spinning = _arg;
    return *this;
  }
  Type & set__is_middle(
    const bool & _arg)
  {
    this->is_middle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    global_interface::msg::Gimbal_<ContainerAllocator> *;
  using ConstRawPtr =
    const global_interface::msg::Gimbal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<global_interface::msg::Gimbal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<global_interface::msg::Gimbal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      global_interface::msg::Gimbal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<global_interface::msg::Gimbal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      global_interface::msg::Gimbal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<global_interface::msg::Gimbal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<global_interface::msg::Gimbal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<global_interface::msg::Gimbal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__global_interface__msg__Gimbal
    std::shared_ptr<global_interface::msg::Gimbal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__global_interface__msg__Gimbal
    std::shared_ptr<global_interface::msg::Gimbal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gimbal_ & other) const
  {
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->is_switched != other.is_switched) {
      return false;
    }
    if (this->is_target != other.is_target) {
      return false;
    }
    if (this->is_spinning != other.is_spinning) {
      return false;
    }
    if (this->is_middle != other.is_middle) {
      return false;
    }
    return true;
  }
  bool operator!=(const Gimbal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gimbal_

// alias to use template instance with default allocator
using Gimbal =
  global_interface::msg::Gimbal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace global_interface

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__GIMBAL__STRUCT_HPP_
