// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parc_robot_interfaces:msg/CropYield.idl
// generated code does not contain a copyright notice

#ifndef PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__TRAITS_HPP_
#define PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parc_robot_interfaces/msg/detail/crop_yield__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace parc_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CropYield & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CropYield & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CropYield & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace parc_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parc_robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parc_robot_interfaces::msg::CropYield & msg,
  std::ostream & out, size_t indentation = 0)
{
  parc_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parc_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const parc_robot_interfaces::msg::CropYield & msg)
{
  return parc_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<parc_robot_interfaces::msg::CropYield>()
{
  return "parc_robot_interfaces::msg::CropYield";
}

template<>
inline const char * name<parc_robot_interfaces::msg::CropYield>()
{
  return "parc_robot_interfaces/msg/CropYield";
}

template<>
struct has_fixed_size<parc_robot_interfaces::msg::CropYield>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<parc_robot_interfaces::msg::CropYield>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<parc_robot_interfaces::msg::CropYield>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__TRAITS_HPP_
