// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from parc_robot_interfaces:msg/CropYield.idl
// generated code does not contain a copyright notice

#ifndef PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "parc_robot_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "parc_robot_interfaces/msg/detail/crop_yield__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace parc_robot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_parc_robot_interfaces
cdr_serialize(
  const parc_robot_interfaces::msg::CropYield & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_parc_robot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  parc_robot_interfaces::msg::CropYield & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_parc_robot_interfaces
get_serialized_size(
  const parc_robot_interfaces::msg::CropYield & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_parc_robot_interfaces
max_serialized_size_CropYield(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace parc_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_parc_robot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, parc_robot_interfaces, msg, CropYield)();

#ifdef __cplusplus
}
#endif

#endif  // PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
