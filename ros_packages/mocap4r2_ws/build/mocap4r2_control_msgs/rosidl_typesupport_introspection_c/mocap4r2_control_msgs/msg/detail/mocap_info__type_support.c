// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mocap4r2_control_msgs:msg/MocapInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mocap4r2_control_msgs/msg/detail/mocap_info__rosidl_typesupport_introspection_c.h"
#include "mocap4r2_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mocap4r2_control_msgs/msg/detail/mocap_info__functions.h"
#include "mocap4r2_control_msgs/msg/detail/mocap_info__struct.h"


// Include directives for member types
// Member `mocap4r2_source`
// Member `topics`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mocap4r2_control_msgs__msg__MocapInfo__init(message_memory);
}

void MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_fini_function(void * message_memory)
{
  mocap4r2_control_msgs__msg__MocapInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_message_member_array[3] = {
  {
    "mocap4r2_source",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_control_msgs__msg__MocapInfo, mocap4r2_source),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ros_version_source",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_control_msgs__msg__MocapInfo, ros_version_source),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "topics",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_control_msgs__msg__MocapInfo, topics),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_message_members = {
  "mocap4r2_control_msgs__msg",  // message namespace
  "MocapInfo",  // message name
  3,  // number of fields
  sizeof(mocap4r2_control_msgs__msg__MocapInfo),
  MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_message_member_array,  // message members
  MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_message_type_support_handle = {
  0,
  &MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mocap4r2_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mocap4r2_control_msgs, msg, MocapInfo)() {
  if (!MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_message_type_support_handle.typesupport_identifier) {
    MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MocapInfo__rosidl_typesupport_introspection_c__MocapInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
