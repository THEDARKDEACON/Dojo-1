// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/SensorData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__SENSOR_DATA__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__SENSOR_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'linear_acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'temperatures'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'temperature_labels'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SensorData in the package robot_interfaces.
/**
  * Unified sensor data message
 */
typedef struct robot_interfaces__msg__SensorData
{
  std_msgs__msg__Header header;
  /// Encoder data
  int32_t left_encoder;
  int32_t right_encoder;
  double left_velocity;
  double right_velocity;
  /// Ultrasonic sensor
  double ultrasonic_distance;
  bool ultrasonic_valid;
  /// IMU data (if available)
  bool imu_available;
  geometry_msgs__msg__Vector3 linear_acceleration;
  geometry_msgs__msg__Vector3 angular_velocity;
  geometry_msgs__msg__Quaternion orientation;
  /// Battery/power info
  double battery_voltage;
  double current_draw;
  /// Temperature sensors
  rosidl_runtime_c__double__Sequence temperatures;
  rosidl_runtime_c__String__Sequence temperature_labels;
} robot_interfaces__msg__SensorData;

// Struct for a sequence of robot_interfaces__msg__SensorData.
typedef struct robot_interfaces__msg__SensorData__Sequence
{
  robot_interfaces__msg__SensorData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__SensorData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__SENSOR_DATA__STRUCT_H_
