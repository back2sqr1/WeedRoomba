// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from parc_robot_interfaces:msg/CropYield.idl
// generated code does not contain a copyright notice

#ifndef PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__FUNCTIONS_H_
#define PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "parc_robot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "parc_robot_interfaces/msg/detail/crop_yield__struct.h"

/// Initialize msg/CropYield message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * parc_robot_interfaces__msg__CropYield
 * )) before or use
 * parc_robot_interfaces__msg__CropYield__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
bool
parc_robot_interfaces__msg__CropYield__init(parc_robot_interfaces__msg__CropYield * msg);

/// Finalize msg/CropYield message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
void
parc_robot_interfaces__msg__CropYield__fini(parc_robot_interfaces__msg__CropYield * msg);

/// Create msg/CropYield message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * parc_robot_interfaces__msg__CropYield__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
parc_robot_interfaces__msg__CropYield *
parc_robot_interfaces__msg__CropYield__create();

/// Destroy msg/CropYield message.
/**
 * It calls
 * parc_robot_interfaces__msg__CropYield__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
void
parc_robot_interfaces__msg__CropYield__destroy(parc_robot_interfaces__msg__CropYield * msg);

/// Check for msg/CropYield message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
bool
parc_robot_interfaces__msg__CropYield__are_equal(const parc_robot_interfaces__msg__CropYield * lhs, const parc_robot_interfaces__msg__CropYield * rhs);

/// Copy a msg/CropYield message.
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
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
bool
parc_robot_interfaces__msg__CropYield__copy(
  const parc_robot_interfaces__msg__CropYield * input,
  parc_robot_interfaces__msg__CropYield * output);

/// Initialize array of msg/CropYield messages.
/**
 * It allocates the memory for the number of elements and calls
 * parc_robot_interfaces__msg__CropYield__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
bool
parc_robot_interfaces__msg__CropYield__Sequence__init(parc_robot_interfaces__msg__CropYield__Sequence * array, size_t size);

/// Finalize array of msg/CropYield messages.
/**
 * It calls
 * parc_robot_interfaces__msg__CropYield__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
void
parc_robot_interfaces__msg__CropYield__Sequence__fini(parc_robot_interfaces__msg__CropYield__Sequence * array);

/// Create array of msg/CropYield messages.
/**
 * It allocates the memory for the array and calls
 * parc_robot_interfaces__msg__CropYield__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
parc_robot_interfaces__msg__CropYield__Sequence *
parc_robot_interfaces__msg__CropYield__Sequence__create(size_t size);

/// Destroy array of msg/CropYield messages.
/**
 * It calls
 * parc_robot_interfaces__msg__CropYield__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
void
parc_robot_interfaces__msg__CropYield__Sequence__destroy(parc_robot_interfaces__msg__CropYield__Sequence * array);

/// Check for msg/CropYield message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
bool
parc_robot_interfaces__msg__CropYield__Sequence__are_equal(const parc_robot_interfaces__msg__CropYield__Sequence * lhs, const parc_robot_interfaces__msg__CropYield__Sequence * rhs);

/// Copy an array of msg/CropYield messages.
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
ROSIDL_GENERATOR_C_PUBLIC_parc_robot_interfaces
bool
parc_robot_interfaces__msg__CropYield__Sequence__copy(
  const parc_robot_interfaces__msg__CropYield__Sequence * input,
  parc_robot_interfaces__msg__CropYield__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__FUNCTIONS_H_
