// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_interfaces:srv/SetMode.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_interfaces/srv/detail/set_mode__rosidl_typesupport_introspection_c.h"
#include "robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_interfaces/srv/detail/set_mode__functions.h"
#include "robot_interfaces/srv/detail/set_mode__struct.h"


// Include directives for member types
// Member `parameters`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_interfaces__srv__SetMode_Request__init(message_memory);
}

void robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_fini_function(void * message_memory)
{
  robot_interfaces__srv__SetMode_Request__fini(message_memory);
}

size_t robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__size_function__SetMode_Request__parameters(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__get_const_function__SetMode_Request__parameters(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__get_function__SetMode_Request__parameters(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__fetch_function__SetMode_Request__parameters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__get_const_function__SetMode_Request__parameters(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__assign_function__SetMode_Request__parameters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__get_function__SetMode_Request__parameters(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__resize_function__SetMode_Request__parameters(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_message_member_array[2] = {
  {
    "requested_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__srv__SetMode_Request, requested_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "parameters",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__srv__SetMode_Request, parameters),  // bytes offset in struct
    NULL,  // default value
    robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__size_function__SetMode_Request__parameters,  // size() function pointer
    robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__get_const_function__SetMode_Request__parameters,  // get_const(index) function pointer
    robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__get_function__SetMode_Request__parameters,  // get(index) function pointer
    robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__fetch_function__SetMode_Request__parameters,  // fetch(index, &value) function pointer
    robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__assign_function__SetMode_Request__parameters,  // assign(index, value) function pointer
    robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__resize_function__SetMode_Request__parameters  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_message_members = {
  "robot_interfaces__srv",  // message namespace
  "SetMode_Request",  // message name
  2,  // number of fields
  sizeof(robot_interfaces__srv__SetMode_Request),
  robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_message_member_array,  // message members
  robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_message_type_support_handle = {
  0,
  &robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, srv, SetMode_Request)() {
  if (!robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_message_type_support_handle.typesupport_identifier) {
    robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_interfaces__srv__SetMode_Request__rosidl_typesupport_introspection_c__SetMode_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_interfaces/srv/detail/set_mode__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_interfaces/srv/detail/set_mode__functions.h"
// already included above
// #include "robot_interfaces/srv/detail/set_mode__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_interfaces__srv__SetMode_Response__init(message_memory);
}

void robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_fini_function(void * message_memory)
{
  robot_interfaces__srv__SetMode_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__srv__SetMode_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__srv__SetMode_Response, current_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__srv__SetMode_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_message_members = {
  "robot_interfaces__srv",  // message namespace
  "SetMode_Response",  // message name
  3,  // number of fields
  sizeof(robot_interfaces__srv__SetMode_Response),
  robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_message_member_array,  // message members
  robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_message_type_support_handle = {
  0,
  &robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, srv, SetMode_Response)() {
  if (!robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_message_type_support_handle.typesupport_identifier) {
    robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_interfaces__srv__SetMode_Response__rosidl_typesupport_introspection_c__SetMode_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robot_interfaces/srv/detail/set_mode__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robot_interfaces__srv__detail__set_mode__rosidl_typesupport_introspection_c__SetMode_service_members = {
  "robot_interfaces__srv",  // service namespace
  "SetMode",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // robot_interfaces__srv__detail__set_mode__rosidl_typesupport_introspection_c__SetMode_Request_message_type_support_handle,
  NULL  // response message
  // robot_interfaces__srv__detail__set_mode__rosidl_typesupport_introspection_c__SetMode_Response_message_type_support_handle
};

static rosidl_service_type_support_t robot_interfaces__srv__detail__set_mode__rosidl_typesupport_introspection_c__SetMode_service_type_support_handle = {
  0,
  &robot_interfaces__srv__detail__set_mode__rosidl_typesupport_introspection_c__SetMode_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, srv, SetMode_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, srv, SetMode_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, srv, SetMode)() {
  if (!robot_interfaces__srv__detail__set_mode__rosidl_typesupport_introspection_c__SetMode_service_type_support_handle.typesupport_identifier) {
    robot_interfaces__srv__detail__set_mode__rosidl_typesupport_introspection_c__SetMode_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robot_interfaces__srv__detail__set_mode__rosidl_typesupport_introspection_c__SetMode_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, srv, SetMode_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, srv, SetMode_Response)()->data;
  }

  return &robot_interfaces__srv__detail__set_mode__rosidl_typesupport_introspection_c__SetMode_service_type_support_handle;
}
