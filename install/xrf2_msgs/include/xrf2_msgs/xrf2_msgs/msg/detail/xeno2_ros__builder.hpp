// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xrf2_msgs:msg/Xeno2Ros.idl
// generated code does not contain a copyright notice

#ifndef XRF2_MSGS__MSG__DETAIL__XENO2_ROS__BUILDER_HPP_
#define XRF2_MSGS__MSG__DETAIL__XENO2_ROS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xrf2_msgs/msg/detail/xeno2_ros__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xrf2_msgs
{

namespace msg
{

namespace builder
{

class Init_Xeno2Ros_theta
{
public:
  explicit Init_Xeno2Ros_theta(::xrf2_msgs::msg::Xeno2Ros & msg)
  : msg_(msg)
  {}
  ::xrf2_msgs::msg::Xeno2Ros theta(::xrf2_msgs::msg::Xeno2Ros::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xrf2_msgs::msg::Xeno2Ros msg_;
};

class Init_Xeno2Ros_y
{
public:
  explicit Init_Xeno2Ros_y(::xrf2_msgs::msg::Xeno2Ros & msg)
  : msg_(msg)
  {}
  Init_Xeno2Ros_theta y(::xrf2_msgs::msg::Xeno2Ros::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Xeno2Ros_theta(msg_);
  }

private:
  ::xrf2_msgs::msg::Xeno2Ros msg_;
};

class Init_Xeno2Ros_x
{
public:
  explicit Init_Xeno2Ros_x(::xrf2_msgs::msg::Xeno2Ros & msg)
  : msg_(msg)
  {}
  Init_Xeno2Ros_y x(::xrf2_msgs::msg::Xeno2Ros::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Xeno2Ros_y(msg_);
  }

private:
  ::xrf2_msgs::msg::Xeno2Ros msg_;
};

class Init_Xeno2Ros_encoder_right
{
public:
  explicit Init_Xeno2Ros_encoder_right(::xrf2_msgs::msg::Xeno2Ros & msg)
  : msg_(msg)
  {}
  Init_Xeno2Ros_x encoder_right(::xrf2_msgs::msg::Xeno2Ros::_encoder_right_type arg)
  {
    msg_.encoder_right = std::move(arg);
    return Init_Xeno2Ros_x(msg_);
  }

private:
  ::xrf2_msgs::msg::Xeno2Ros msg_;
};

class Init_Xeno2Ros_encoder_left
{
public:
  Init_Xeno2Ros_encoder_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Xeno2Ros_encoder_right encoder_left(::xrf2_msgs::msg::Xeno2Ros::_encoder_left_type arg)
  {
    msg_.encoder_left = std::move(arg);
    return Init_Xeno2Ros_encoder_right(msg_);
  }

private:
  ::xrf2_msgs::msg::Xeno2Ros msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::xrf2_msgs::msg::Xeno2Ros>()
{
  return xrf2_msgs::msg::builder::Init_Xeno2Ros_encoder_left();
}

}  // namespace xrf2_msgs

#endif  // XRF2_MSGS__MSG__DETAIL__XENO2_ROS__BUILDER_HPP_
