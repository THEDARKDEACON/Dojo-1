// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/HardwareStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__HARDWARE_STATUS__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__HARDWARE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'UNKNOWN'.
/**
  * Status information
 */
enum
{
  robot_interfaces__msg__HardwareStatus__UNKNOWN = 0
};

/// Constant 'INITIALIZING'.
enum
{
  robot_interfaces__msg__HardwareStatus__INITIALIZING = 1
};

/// Constant 'READY'.
enum
{
  robot_interfaces__msg__HardwareStatus__READY = 2
};

/// Constant 'ERROR'.
enum
{
  robot_interfaces__msg__HardwareStatus__ERROR = 3
};

/// Constant 'DISCONNECTED'.
enum
{
  robot_interfaces__msg__HardwareStatus__DISCONNECTED = 4
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'component_name'
// Member 'hardware_id'
// Member 'status_message'
// Member 'data_keys'
// Member 'data_values'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/HardwareStatus in the package robot_interfaces.
/**
  * Hardware component status message
 */
typedef struct robot_interfaces__msg__HardwareStatus
{
  std_msgs__msg__Header header;
  /// Component identifiers
  rosidl_runtime_c__String component_name;
  rosidl_runtime_c__String hardware_id;
  uint8_t status;
  rosidl_runtime_c__String status_message;
  double uptime;
  double last_update;
  /// Component-specific data
  rosidl_runtime_c__String__Sequence data_keys;
  rosidl_runtime_c__String__Sequence data_values;
} robot_interfaces__msg__HardwareStatus;

// Struct for a sequence of robot_interfaces__msg__HardwareStatus.
typedef struct robot_interfaces__msg__HardwareStatus__Sequence
{
  robot_interfaces__msg__HardwareStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__HardwareStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__HARDWARE_STATUS__STRUCT_H_
