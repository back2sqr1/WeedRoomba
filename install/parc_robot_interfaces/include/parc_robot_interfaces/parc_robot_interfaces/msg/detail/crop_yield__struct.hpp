// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parc_robot_interfaces:msg/CropYield.idl
// generated code does not contain a copyright notice

#ifndef PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__STRUCT_HPP_
#define PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__parc_robot_interfaces__msg__CropYield __attribute__((deprecated))
#else
# define DEPRECATED__parc_robot_interfaces__msg__CropYield __declspec(deprecated)
#endif

namespace parc_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CropYield_
{
  using Type = CropYield_<ContainerAllocator>;

  explicit CropYield_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0ll;
    }
  }

  explicit CropYield_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0ll;
    }
  }

  // field types and members
  using _data_type =
    int64_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const int64_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parc_robot_interfaces::msg::CropYield_<ContainerAllocator> *;
  using ConstRawPtr =
    const parc_robot_interfaces::msg::CropYield_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parc_robot_interfaces::msg::CropYield_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parc_robot_interfaces::msg::CropYield_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parc_robot_interfaces::msg::CropYield_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parc_robot_interfaces::msg::CropYield_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parc_robot_interfaces::msg::CropYield_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parc_robot_interfaces::msg::CropYield_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parc_robot_interfaces::msg::CropYield_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parc_robot_interfaces::msg::CropYield_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parc_robot_interfaces__msg__CropYield
    std::shared_ptr<parc_robot_interfaces::msg::CropYield_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parc_robot_interfaces__msg__CropYield
    std::shared_ptr<parc_robot_interfaces::msg::CropYield_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CropYield_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const CropYield_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CropYield_

// alias to use template instance with default allocator
using CropYield =
  parc_robot_interfaces::msg::CropYield_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace parc_robot_interfaces

#endif  // PARC_ROBOT_INTERFACES__MSG__DETAIL__CROP_YIELD__STRUCT_HPP_
