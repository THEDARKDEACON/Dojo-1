// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'VELOCITY_MODE'.
/**
  * Motor control modes
 */
enum
{
  robot_interfaces__msg__MotorCommand__VELOCITY_MODE = 0
};

/// Constant 'PWM_MODE'.
enum
{
  robot_interfaces__msg__MotorCommand__PWM_MODE = 1
};

/// Constant 'POSITION_MODE'.
enum
{
  robot_interfaces__msg__MotorCommand__POSITION_MODE = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/MotorCommand in the package robot_interfaces.
/**
  * Motor command message
 */
typedef struct robot_interfaces__msg__MotorCommand
{
  std_msgs__msg__Header header;
  uint8_t control_mode;
  /// Velocity commands (m/s)
  double left_velocity;
  double right_velocity;
  /// PWM commands (-255 to 255)
  int16_t left_pwm;
  int16_t right_pwm;
  /// Position commands (radians)
  double left_position;
  double right_position;
  /// Safety limits
  double max_velocity;
  double max_acceleration;
  bool emergency_stop;
} robot_interfaces__msg__MotorCommand;

// Struct for a sequence of robot_interfaces__msg__MotorCommand.
typedef struct robot_interfaces__msg__MotorCommand__Sequence
{
  robot_interfaces__msg__MotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__MotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
