// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:srv/SetMode.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__SRV__DETAIL__SET_MODE__STRUCT_H_
#define ROBOT_INTERFACES__SRV__DETAIL__SET_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IDLE'.
/**
  * Request
 */
enum
{
  robot_interfaces__srv__SetMode_Request__IDLE = 0
};

/// Constant 'MANUAL'.
enum
{
  robot_interfaces__srv__SetMode_Request__MANUAL = 1
};

/// Constant 'AUTONOMOUS'.
enum
{
  robot_interfaces__srv__SetMode_Request__AUTONOMOUS = 2
};

/// Constant 'CALIBRATION'.
enum
{
  robot_interfaces__srv__SetMode_Request__CALIBRATION = 3
};

// Include directives for member types
// Member 'parameters'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetMode in the package robot_interfaces.
typedef struct robot_interfaces__srv__SetMode_Request
{
  uint8_t requested_mode;
  rosidl_runtime_c__String__Sequence parameters;
} robot_interfaces__srv__SetMode_Request;

// Struct for a sequence of robot_interfaces__srv__SetMode_Request.
typedef struct robot_interfaces__srv__SetMode_Request__Sequence
{
  robot_interfaces__srv__SetMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__srv__SetMode_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetMode in the package robot_interfaces.
typedef struct robot_interfaces__srv__SetMode_Response
{
  /// Response
  bool success;
  uint8_t current_mode;
  rosidl_runtime_c__String message;
} robot_interfaces__srv__SetMode_Response;

// Struct for a sequence of robot_interfaces__srv__SetMode_Response.
typedef struct robot_interfaces__srv__SetMode_Response__Sequence
{
  robot_interfaces__srv__SetMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__srv__SetMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__SRV__DETAIL__SET_MODE__STRUCT_H_
