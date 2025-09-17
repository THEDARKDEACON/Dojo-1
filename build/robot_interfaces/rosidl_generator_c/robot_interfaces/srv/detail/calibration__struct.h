// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:srv/Calibration.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__SRV__DETAIL__CALIBRATION__STRUCT_H_
#define ROBOT_INTERFACES__SRV__DETAIL__CALIBRATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'component_name'
// Member 'calibration_type'
// Member 'parameters'
#include "rosidl_runtime_c/string.h"
// Member 'values'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/Calibration in the package robot_interfaces.
typedef struct robot_interfaces__srv__Calibration_Request
{
  /// Request
  rosidl_runtime_c__String component_name;
  rosidl_runtime_c__String calibration_type;
  rosidl_runtime_c__String__Sequence parameters;
  rosidl_runtime_c__double__Sequence values;
} robot_interfaces__srv__Calibration_Request;

// Struct for a sequence of robot_interfaces__srv__Calibration_Request.
typedef struct robot_interfaces__srv__Calibration_Request__Sequence
{
  robot_interfaces__srv__Calibration_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__srv__Calibration_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'calibration_results'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/Calibration in the package robot_interfaces.
typedef struct robot_interfaces__srv__Calibration_Response
{
  /// Response
  bool success;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__double__Sequence calibration_results;
} robot_interfaces__srv__Calibration_Response;

// Struct for a sequence of robot_interfaces__srv__Calibration_Response.
typedef struct robot_interfaces__srv__Calibration_Response__Sequence
{
  robot_interfaces__srv__Calibration_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__srv__Calibration_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__SRV__DETAIL__CALIBRATION__STRUCT_H_
