// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parc_robot_interfaces:msg/CropYield.idl
// generated code does not contain a copyright notice

#ifndef PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__STRUCT_H_
#define PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CropYield in the package parc_robot_interfaces.
typedef struct parc_robot_interfaces__msg__CropYield
{
  int64_t data;
} parc_robot_interfaces__msg__CropYield;

// Struct for a sequence of parc_robot_interfaces__msg__CropYield.
typedef struct parc_robot_interfaces__msg__CropYield__Sequence
{
  parc_robot_interfaces__msg__CropYield * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parc_robot_interfaces__msg__CropYield__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__STRUCT_H_
