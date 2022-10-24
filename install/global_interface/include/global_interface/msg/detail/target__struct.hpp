// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from global_interface:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__TARGET__STRUCT_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__TARGET__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'aiming_point'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'rmat_imu'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__global_interface__msg__Target __attribute__((deprecated))
#else
# define DEPRECATED__global_interface__msg__Target __declspec(deprecated)
#endif

namespace global_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Target_
{
  using Type = Target_<ContainerAllocator>;

  explicit Target_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : aiming_point(_init),
    rmat_imu(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0.0;
    }
  }

  explicit Target_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : aiming_point(_alloc, _init),
    rmat_imu(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0.0;
    }
  }

  // field types and members
  using _aiming_point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _aiming_point_type aiming_point;
  using _timestamp_type =
    double;
  _timestamp_type timestamp;
  using _rmat_imu_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _rmat_imu_type rmat_imu;

  // setters for named parameter idiom
  Type & set__aiming_point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->aiming_point = _arg;
    return *this;
  }
  Type & set__timestamp(
    const double & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__rmat_imu(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->rmat_imu = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    global_interface::msg::Target_<ContainerAllocator> *;
  using ConstRawPtr =
    const global_interface::msg::Target_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<global_interface::msg::Target_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<global_interface::msg::Target_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      global_interface::msg::Target_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<global_interface::msg::Target_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      global_interface::msg::Target_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<global_interface::msg::Target_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<global_interface::msg::Target_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<global_interface::msg::Target_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__global_interface__msg__Target
    std::shared_ptr<global_interface::msg::Target_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__global_interface__msg__Target
    std::shared_ptr<global_interface::msg::Target_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Target_ & other) const
  {
    if (this->aiming_point != other.aiming_point) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->rmat_imu != other.rmat_imu) {
      return false;
    }
    return true;
  }
  bool operator!=(const Target_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Target_

// alias to use template instance with default allocator
using Target =
  global_interface::msg::Target_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace global_interface

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__TARGET__STRUCT_HPP_
