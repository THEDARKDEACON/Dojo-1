// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/SensorData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__SENSOR_DATA__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__SENSOR_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/sensor_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_SensorData_temperature_labels
{
public:
  explicit Init_SensorData_temperature_labels(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::SensorData temperature_labels(::robot_interfaces::msg::SensorData::_temperature_labels_type arg)
  {
    msg_.temperature_labels = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_temperatures
{
public:
  explicit Init_SensorData_temperatures(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_temperature_labels temperatures(::robot_interfaces::msg::SensorData::_temperatures_type arg)
  {
    msg_.temperatures = std::move(arg);
    return Init_SensorData_temperature_labels(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_current_draw
{
public:
  explicit Init_SensorData_current_draw(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_temperatures current_draw(::robot_interfaces::msg::SensorData::_current_draw_type arg)
  {
    msg_.current_draw = std::move(arg);
    return Init_SensorData_temperatures(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_battery_voltage
{
public:
  explicit Init_SensorData_battery_voltage(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_current_draw battery_voltage(::robot_interfaces::msg::SensorData::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_SensorData_current_draw(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_orientation
{
public:
  explicit Init_SensorData_orientation(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_battery_voltage orientation(::robot_interfaces::msg::SensorData::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_SensorData_battery_voltage(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_angular_velocity
{
public:
  explicit Init_SensorData_angular_velocity(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_orientation angular_velocity(::robot_interfaces::msg::SensorData::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_SensorData_orientation(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_linear_acceleration
{
public:
  explicit Init_SensorData_linear_acceleration(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_angular_velocity linear_acceleration(::robot_interfaces::msg::SensorData::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return Init_SensorData_angular_velocity(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_imu_available
{
public:
  explicit Init_SensorData_imu_available(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_linear_acceleration imu_available(::robot_interfaces::msg::SensorData::_imu_available_type arg)
  {
    msg_.imu_available = std::move(arg);
    return Init_SensorData_linear_acceleration(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_ultrasonic_valid
{
public:
  explicit Init_SensorData_ultrasonic_valid(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_imu_available ultrasonic_valid(::robot_interfaces::msg::SensorData::_ultrasonic_valid_type arg)
  {
    msg_.ultrasonic_valid = std::move(arg);
    return Init_SensorData_imu_available(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_ultrasonic_distance
{
public:
  explicit Init_SensorData_ultrasonic_distance(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_ultrasonic_valid ultrasonic_distance(::robot_interfaces::msg::SensorData::_ultrasonic_distance_type arg)
  {
    msg_.ultrasonic_distance = std::move(arg);
    return Init_SensorData_ultrasonic_valid(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_right_velocity
{
public:
  explicit Init_SensorData_right_velocity(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_ultrasonic_distance right_velocity(::robot_interfaces::msg::SensorData::_right_velocity_type arg)
  {
    msg_.right_velocity = std::move(arg);
    return Init_SensorData_ultrasonic_distance(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_left_velocity
{
public:
  explicit Init_SensorData_left_velocity(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_right_velocity left_velocity(::robot_interfaces::msg::SensorData::_left_velocity_type arg)
  {
    msg_.left_velocity = std::move(arg);
    return Init_SensorData_right_velocity(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_right_encoder
{
public:
  explicit Init_SensorData_right_encoder(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_left_velocity right_encoder(::robot_interfaces::msg::SensorData::_right_encoder_type arg)
  {
    msg_.right_encoder = std::move(arg);
    return Init_SensorData_left_velocity(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_left_encoder
{
public:
  explicit Init_SensorData_left_encoder(::robot_interfaces::msg::SensorData & msg)
  : msg_(msg)
  {}
  Init_SensorData_right_encoder left_encoder(::robot_interfaces::msg::SensorData::_left_encoder_type arg)
  {
    msg_.left_encoder = std::move(arg);
    return Init_SensorData_right_encoder(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

class Init_SensorData_header
{
public:
  Init_SensorData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorData_left_encoder header(::robot_interfaces::msg::SensorData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SensorData_left_encoder(msg_);
  }

private:
  ::robot_interfaces::msg::SensorData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::SensorData>()
{
  return robot_interfaces::msg::builder::Init_SensorData_header();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__SENSOR_DATA__BUILDER_HPP_
