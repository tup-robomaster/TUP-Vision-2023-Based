// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from global_interface:msg/Armors.idl
// generated code does not contain a copyright notice

#ifndef GLOBAL_INTERFACE__MSG__DETAIL__ARMORS__FUNCTIONS_H_
#define GLOBAL_INTERFACE__MSG__DETAIL__ARMORS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "global_interface/msg/rosidl_generator_c__visibility_control.h"

#include "global_interface/msg/detail/armors__struct.h"

/// Initialize msg/Armors message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * global_interface__msg__Armors
 * )) before or use
 * global_interface__msg__Armors__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
bool
global_interface__msg__Armors__init(global_interface__msg__Armors * msg);

/// Finalize msg/Armors message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
void
global_interface__msg__Armors__fini(global_interface__msg__Armors * msg);

/// Create msg/Armors message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * global_interface__msg__Armors__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
global_interface__msg__Armors *
global_interface__msg__Armors__create();

/// Destroy msg/Armors message.
/**
 * It calls
 * global_interface__msg__Armors__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
void
global_interface__msg__Armors__destroy(global_interface__msg__Armors * msg);

/// Check for msg/Armors message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
bool
global_interface__msg__Armors__are_equal(const global_interface__msg__Armors * lhs, const global_interface__msg__Armors * rhs);

/// Copy a msg/Armors message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
bool
global_interface__msg__Armors__copy(
  const global_interface__msg__Armors * input,
  global_interface__msg__Armors * output);

/// Initialize array of msg/Armors messages.
/**
 * It allocates the memory for the number of elements and calls
 * global_interface__msg__Armors__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
bool
global_interface__msg__Armors__Sequence__init(global_interface__msg__Armors__Sequence * array, size_t size);

/// Finalize array of msg/Armors messages.
/**
 * It calls
 * global_interface__msg__Armors__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
void
global_interface__msg__Armors__Sequence__fini(global_interface__msg__Armors__Sequence * array);

/// Create array of msg/Armors messages.
/**
 * It allocates the memory for the array and calls
 * global_interface__msg__Armors__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
global_interface__msg__Armors__Sequence *
global_interface__msg__Armors__Sequence__create(size_t size);

/// Destroy array of msg/Armors messages.
/**
 * It calls
 * global_interface__msg__Armors__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
void
global_interface__msg__Armors__Sequence__destroy(global_interface__msg__Armors__Sequence * array);

/// Check for msg/Armors message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
bool
global_interface__msg__Armors__Sequence__are_equal(const global_interface__msg__Armors__Sequence * lhs, const global_interface__msg__Armors__Sequence * rhs);

/// Copy an array of msg/Armors messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_global_interface
bool
global_interface__msg__Armors__Sequence__copy(
  const global_interface__msg__Armors__Sequence * input,
  global_interface__msg__Armors__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GLOBAL_INTERFACE__MSG__DETAIL__ARMORS__FUNCTIONS_H_
