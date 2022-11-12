// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from global_interface:msg/Armors.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__ARMORS__STRUCT_HPP_
#define GLOBAL_INTERFACE__MSG__DETAIL__ARMORS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'armors'
#include "global_interface/msg/detail/armor__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__global_interface__msg__Armors __attribute__((deprecated))
#else
# define DEPRECATED__global_interface__msg__Armors __declspec(deprecated)
#endif

namespace global_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Armors_
{
  using Type = Armors_<ContainerAllocator>;

  explicit Armors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Armors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _armors_type =
    std::vector<global_interface::msg::Armor_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<global_interface::msg::Armor_<ContainerAllocator>>>;
  _armors_type armors;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__armors(
    const std::vector<global_interface::msg::Armor_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<global_interface::msg::Armor_<ContainerAllocator>>> & _arg)
  {
    this->armors = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    global_interface::msg::Armors_<ContainerAllocator> *;
  using ConstRawPtr =
    const global_interface::msg::Armors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<global_interface::msg::Armors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<global_interface::msg::Armors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      global_interface::msg::Armors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<global_interface::msg::Armors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      global_interface::msg::Armors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<global_interface::msg::Armors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<global_interface::msg::Armors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<global_interface::msg::Armors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__global_interface__msg__Armors
    std::shared_ptr<global_interface::msg::Armors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__global_interface__msg__Armors
    std::shared_ptr<global_interface::msg::Armors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Armors_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->armors != other.armors) {
      return false;
    }
    return true;
  }
  bool operator!=(const Armors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Armors_

// alias to use template instance with default allocator
using Armors =
  global_interface::msg::Armors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace global_interface

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__ARMORS__STRUCT_HPP_
