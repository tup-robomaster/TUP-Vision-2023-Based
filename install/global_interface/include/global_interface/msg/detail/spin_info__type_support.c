// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from global_interface:msg/SpinInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "global_interface/msg/detail/spin_info__rosidl_typesupport_introspection_c.h"
#include "global_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "global_interface/msg/detail/spin_info__functions.h"
#include "global_interface/msg/detail/spin_info__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  global_interface__msg__SpinInfo__init(message_memory);
}

void SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_fini_function(void * message_memory)
{
  global_interface__msg__SpinInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(global_interface__msg__SpinInfo, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_spinning",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(global_interface__msg__SpinInfo, is_spinning),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_message_members = {
  "global_interface__msg",  // message namespace
  "SpinInfo",  // message name
  2,  // number of fields
  sizeof(global_interface__msg__SpinInfo),
  SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_message_member_array,  // message members
  SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_message_type_support_handle = {
  0,
  &SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_global_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, global_interface, msg, SpinInfo)() {
  SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_message_type_support_handle.typesupport_identifier) {
    SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SpinInfo__rosidl_typesupport_introspection_c__SpinInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
