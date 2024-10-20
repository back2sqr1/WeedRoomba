// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parc_robot_interfaces:msg/CropYield.idl
// generated code does not contain a copyright notice

#ifndef PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__BUILDER_HPP_
#define PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parc_robot_interfaces/msg/detail/crop_yield__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parc_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_CropYield_data
{
public:
  Init_CropYield_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parc_robot_interfaces::msg::CropYield data(::parc_robot_interfaces::msg::CropYield::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parc_robot_interfaces::msg::CropYield msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::parc_robot_interfaces::msg::CropYield>()
{
  return parc_robot_interfaces::msg::builder::Init_CropYield_data();
}

}  // namespace parc_robot_interfaces

#endif  // PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__BUILDER_HPP_
