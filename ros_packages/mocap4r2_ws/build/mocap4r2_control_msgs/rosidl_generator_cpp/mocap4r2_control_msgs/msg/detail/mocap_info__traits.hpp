// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mocap4r2_control_msgs:msg/MocapInfo.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_CONTROL_MSGS__MSG__DETAIL__MOCAP_INFO__TRAITS_HPP_
#define MOCAP4R2_CONTROL_MSGS__MSG__DETAIL__MOCAP_INFO__TRAITS_HPP_

#include "mocap4r2_control_msgs/msg/detail/mocap_info__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mocap4r2_control_msgs::msg::MocapInfo>()
{
  return "mocap4r2_control_msgs::msg::MocapInfo";
}

template<>
inline const char * name<mocap4r2_control_msgs::msg::MocapInfo>()
{
  return "mocap4r2_control_msgs/msg/MocapInfo";
}

template<>
struct has_fixed_size<mocap4r2_control_msgs::msg::MocapInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mocap4r2_control_msgs::msg::MocapInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mocap4r2_control_msgs::msg::MocapInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOCAP4R2_CONTROL_MSGS__MSG__DETAIL__MOCAP_INFO__TRAITS_HPP_
