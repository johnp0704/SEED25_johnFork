// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mocap4r2_msgs:msg/RigidBody.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODY__TRAITS_HPP_
#define MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODY__TRAITS_HPP_

#include "mocap4r2_msgs/msg/detail/rigid_body__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mocap4r2_msgs::msg::RigidBody>()
{
  return "mocap4r2_msgs::msg::RigidBody";
}

template<>
inline const char * name<mocap4r2_msgs::msg::RigidBody>()
{
  return "mocap4r2_msgs/msg/RigidBody";
}

template<>
struct has_fixed_size<mocap4r2_msgs::msg::RigidBody>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mocap4r2_msgs::msg::RigidBody>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mocap4r2_msgs::msg::RigidBody>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODY__TRAITS_HPP_
