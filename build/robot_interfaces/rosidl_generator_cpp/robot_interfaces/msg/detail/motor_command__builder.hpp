// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorCommand_emergency_stop
{
public:
  explicit Init_MotorCommand_emergency_stop(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::MotorCommand emergency_stop(::robot_interfaces::msg::MotorCommand::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_max_acceleration
{
public:
  explicit Init_MotorCommand_max_acceleration(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_emergency_stop max_acceleration(::robot_interfaces::msg::MotorCommand::_max_acceleration_type arg)
  {
    msg_.max_acceleration = std::move(arg);
    return Init_MotorCommand_emergency_stop(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_max_velocity
{
public:
  explicit Init_MotorCommand_max_velocity(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_max_acceleration max_velocity(::robot_interfaces::msg::MotorCommand::_max_velocity_type arg)
  {
    msg_.max_velocity = std::move(arg);
    return Init_MotorCommand_max_acceleration(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_right_position
{
public:
  explicit Init_MotorCommand_right_position(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_max_velocity right_position(::robot_interfaces::msg::MotorCommand::_right_position_type arg)
  {
    msg_.right_position = std::move(arg);
    return Init_MotorCommand_max_velocity(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_left_position
{
public:
  explicit Init_MotorCommand_left_position(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_right_position left_position(::robot_interfaces::msg::MotorCommand::_left_position_type arg)
  {
    msg_.left_position = std::move(arg);
    return Init_MotorCommand_right_position(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_right_pwm
{
public:
  explicit Init_MotorCommand_right_pwm(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_left_position right_pwm(::robot_interfaces::msg::MotorCommand::_right_pwm_type arg)
  {
    msg_.right_pwm = std::move(arg);
    return Init_MotorCommand_left_position(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_left_pwm
{
public:
  explicit Init_MotorCommand_left_pwm(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_right_pwm left_pwm(::robot_interfaces::msg::MotorCommand::_left_pwm_type arg)
  {
    msg_.left_pwm = std::move(arg);
    return Init_MotorCommand_right_pwm(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_right_velocity
{
public:
  explicit Init_MotorCommand_right_velocity(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_left_pwm right_velocity(::robot_interfaces::msg::MotorCommand::_right_velocity_type arg)
  {
    msg_.right_velocity = std::move(arg);
    return Init_MotorCommand_left_pwm(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_left_velocity
{
public:
  explicit Init_MotorCommand_left_velocity(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_right_velocity left_velocity(::robot_interfaces::msg::MotorCommand::_left_velocity_type arg)
  {
    msg_.left_velocity = std::move(arg);
    return Init_MotorCommand_right_velocity(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_control_mode
{
public:
  explicit Init_MotorCommand_control_mode(::robot_interfaces::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_left_velocity control_mode(::robot_interfaces::msg::MotorCommand::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_MotorCommand_left_velocity(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

class Init_MotorCommand_header
{
public:
  Init_MotorCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_control_mode header(::robot_interfaces::msg::MotorCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorCommand_control_mode(msg_);
  }

private:
  ::robot_interfaces::msg::MotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::MotorCommand>()
{
  return robot_interfaces::msg::builder::Init_MotorCommand_header();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
