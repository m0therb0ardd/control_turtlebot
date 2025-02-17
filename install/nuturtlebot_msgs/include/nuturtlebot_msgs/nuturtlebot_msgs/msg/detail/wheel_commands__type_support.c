// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from nuturtlebot_msgs:msg/WheelCommands.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "nuturtlebot_msgs/msg/detail/wheel_commands__rosidl_typesupport_introspection_c.h"
#include "nuturtlebot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "nuturtlebot_msgs/msg/detail/wheel_commands__functions.h"
#include "nuturtlebot_msgs/msg/detail/wheel_commands__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  nuturtlebot_msgs__msg__WheelCommands__init(message_memory);
}

void nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_fini_function(void * message_memory)
{
  nuturtlebot_msgs__msg__WheelCommands__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_message_member_array[2] = {
  {
    "left_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nuturtlebot_msgs__msg__WheelCommands, left_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nuturtlebot_msgs__msg__WheelCommands, right_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_message_members = {
  "nuturtlebot_msgs__msg",  // message namespace
  "WheelCommands",  // message name
  2,  // number of fields
  sizeof(nuturtlebot_msgs__msg__WheelCommands),
  false,  // has_any_key_member_
  nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_message_member_array,  // message members
  nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_init_function,  // function to initialize message memory (memory has to be allocated)
  nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_message_type_support_handle = {
  0,
  &nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_message_members,
  get_message_typesupport_handle_function,
  &nuturtlebot_msgs__msg__WheelCommands__get_type_hash,
  &nuturtlebot_msgs__msg__WheelCommands__get_type_description,
  &nuturtlebot_msgs__msg__WheelCommands__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nuturtlebot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nuturtlebot_msgs, msg, WheelCommands)() {
  if (!nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_message_type_support_handle.typesupport_identifier) {
    nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &nuturtlebot_msgs__msg__WheelCommands__rosidl_typesupport_introspection_c__WheelCommands_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
