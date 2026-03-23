// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mocap4r2_robot_gt_msgs:srv/SetGTOrigin.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_ROBOT_GT_MSGS__SRV__DETAIL__SET_GT_ORIGIN__TRAITS_HPP_
#define MOCAP4R2_ROBOT_GT_MSGS__SRV__DETAIL__SET_GT_ORIGIN__TRAITS_HPP_

#include "mocap4r2_robot_gt_msgs/srv/detail/set_gt_origin__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'origin_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Request>()
{
  return "mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Request";
}

template<>
inline const char * name<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Request>()
{
  return "mocap4r2_robot_gt_msgs/srv/SetGTOrigin_Request";
}

template<>
struct has_fixed_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Response>()
{
  return "mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Response";
}

template<>
inline const char * name<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Response>()
{
  return "mocap4r2_robot_gt_msgs/srv/SetGTOrigin_Response";
}

template<>
struct has_fixed_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mocap4r2_robot_gt_msgs::srv::SetGTOrigin>()
{
  return "mocap4r2_robot_gt_msgs::srv::SetGTOrigin";
}

template<>
inline const char * name<mocap4r2_robot_gt_msgs::srv::SetGTOrigin>()
{
  return "mocap4r2_robot_gt_msgs/srv/SetGTOrigin";
}

template<>
struct has_fixed_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin>
  : std::integral_constant<
    bool,
    has_fixed_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Request>::value &&
    has_fixed_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Response>::value
  >
{
};

template<>
struct has_bounded_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin>
  : std::integral_constant<
    bool,
    has_bounded_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Request>::value &&
    has_bounded_size<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Response>::value
  >
{
};

template<>
struct is_service<mocap4r2_robot_gt_msgs::srv::SetGTOrigin>
  : std::true_type
{
};

template<>
struct is_service_request<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mocap4r2_robot_gt_msgs::srv::SetGTOrigin_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MOCAP4R2_ROBOT_GT_MSGS__SRV__DETAIL__SET_GT_ORIGIN__TRAITS_HPP_
