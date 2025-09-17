// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

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
  * Robot operational states
 */
enum
{
  robot_interfaces__msg__RobotState__UNKNOWN = 0
};

/// Constant 'INITIALIZING'.
enum
{
  robot_interfaces__msg__RobotState__INITIALIZING = 1
};

/// Constant 'IDLE'.
enum
{
  robot_interfaces__msg__RobotState__IDLE = 2
};

/// Constant 'MANUAL_CONTROL'.
enum
{
  robot_interfaces__msg__RobotState__MANUAL_CONTROL = 3
};

/// Constant 'AUTONOMOUS'.
enum
{
  robot_interfaces__msg__RobotState__AUTONOMOUS = 4
};

/// Constant 'ERROR'.
enum
{
  robot_interfaces__msg__RobotState__ERROR = 5
};

/// Constant 'EMERGENCY_STOP'.
enum
{
  robot_interfaces__msg__RobotState__EMERGENCY_STOP = 6
};

/// Constant 'SHUTDOWN'.
enum
{
  robot_interfaces__msg__RobotState__SHUTDOWN = 7
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'state_change_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'active_errors'
// Member 'warnings'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotState in the package robot_interfaces.
/**
  * Overall robot state message
 */
typedef struct robot_interfaces__msg__RobotState
{
  std_msgs__msg__Header header;
  uint8_t current_state;
  uint8_t previous_state;
  builtin_interfaces__msg__Time state_change_time;
  /// System health
  bool hardware_ok;
  bool software_ok;
  bool communication_ok;
  bool sensors_ok;
  /// Current capabilities
  bool can_move;
  bool can_navigate;
  bool can_perceive;
  /// Error information
  rosidl_runtime_c__String__Sequence active_errors;
  rosidl_runtime_c__String__Sequence warnings;
  /// Performance metrics
  double cpu_usage;
  double memory_usage;
  double network_usage;
} robot_interfaces__msg__RobotState;

// Struct for a sequence of robot_interfaces__msg__RobotState.
typedef struct robot_interfaces__msg__RobotState__Sequence
{
  robot_interfaces__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
