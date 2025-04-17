// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xrf2_msgs:msg/Xeno2Ros.idl
// generated code does not contain a copyright notice

#ifndef XRF2_MSGS__MSG__DETAIL__XENO2_ROS__STRUCT_HPP_
#define XRF2_MSGS__MSG__DETAIL__XENO2_ROS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__xrf2_msgs__msg__Xeno2Ros __attribute__((deprecated))
#else
# define DEPRECATED__xrf2_msgs__msg__Xeno2Ros __declspec(deprecated)
#endif

namespace xrf2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Xeno2Ros_
{
  using Type = Xeno2Ros_<ContainerAllocator>;

  explicit Xeno2Ros_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encoder_left = 0.0;
      this->encoder_right = 0.0;
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  explicit Xeno2Ros_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encoder_left = 0.0;
      this->encoder_right = 0.0;
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  // field types and members
  using _encoder_left_type =
    double;
  _encoder_left_type encoder_left;
  using _encoder_right_type =
    double;
  _encoder_right_type encoder_right;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _theta_type =
    double;
  _theta_type theta;

  // setters for named parameter idiom
  Type & set__encoder_left(
    const double & _arg)
  {
    this->encoder_left = _arg;
    return *this;
  }
  Type & set__encoder_right(
    const double & _arg)
  {
    this->encoder_right = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator> *;
  using ConstRawPtr =
    const xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xrf2_msgs__msg__Xeno2Ros
    std::shared_ptr<xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xrf2_msgs__msg__Xeno2Ros
    std::shared_ptr<xrf2_msgs::msg::Xeno2Ros_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Xeno2Ros_ & other) const
  {
    if (this->encoder_left != other.encoder_left) {
      return false;
    }
    if (this->encoder_right != other.encoder_right) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    return true;
  }
  bool operator!=(const Xeno2Ros_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Xeno2Ros_

// alias to use template instance with default allocator
using Xeno2Ros =
  xrf2_msgs::msg::Xeno2Ros_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace xrf2_msgs

#endif  // XRF2_MSGS__MSG__DETAIL__XENO2_ROS__STRUCT_HPP_
