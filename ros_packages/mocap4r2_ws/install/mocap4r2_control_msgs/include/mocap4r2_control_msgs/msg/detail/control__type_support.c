// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mocap4r2_control_msgs:msg/Control.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mocap4r2_control_msgs/msg/detail/control__rosidl_typesupport_introspection_c.h"
#include "mocap4r2_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mocap4r2_control_msgs/msg/detail/control__functions.h"
#include "mocap4r2_control_msgs/msg/detail/control__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `mocap4r2_source`
// Member `session_id`
// Member `mocap4r2_systems`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Control__rosidl_typesupport_introspection_c__Control_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mocap4r2_control_msgs__msg__Control__init(message_memory);
}

void Control__rosidl_typesupport_introspection_c__Control_fini_function(void * message_memory)
{
  mocap4r2_control_msgs__msg__Control__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Control__rosidl_typesupport_introspection_c__Control_message_member_array[5] = {
  {
    "control_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_control_msgs__msg__Control, control_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_control_msgs__msg__Control, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mocap4r2_source",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_control_msgs__msg__Control, mocap4r2_source),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "session_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_control_msgs__msg__Control, session_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mocap4r2_systems",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_control_msgs__msg__Control, mocap4r2_systems),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Control__rosidl_typesupport_introspection_c__Control_message_members = {
  "mocap4r2_control_msgs__msg",  // message namespace
  "Control",  // message name
  5,  // number of fields
  sizeof(mocap4r2_control_msgs__msg__Control),
  Control__rosidl_typesupport_introspection_c__Control_message_member_array,  // message members
  Control__rosidl_typesupport_introspection_c__Control_init_function,  // function to initialize message memory (memory has to be allocated)
  Control__rosidl_typesupport_introspection_c__Control_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Control__rosidl_typesupport_introspection_c__Control_message_type_support_handle = {
  0,
  &Control__rosidl_typesupport_introspection_c__Control_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mocap4r2_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mocap4r2_control_msgs, msg, Control)() {
  Control__rosidl_typesupport_introspection_c__Control_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!Control__rosidl_typesupport_introspection_c__Control_message_type_support_handle.typesupport_identifier) {
    Control__rosidl_typesupport_introspection_c__Control_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Control__rosidl_typesupport_introspection_c__Control_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
