// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotState_network_usage
{
public:
  explicit Init_RobotState_network_usage(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::RobotState network_usage(::robot_interfaces::msg::RobotState::_network_usage_type arg)
  {
    msg_.network_usage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_memory_usage
{
public:
  explicit Init_RobotState_memory_usage(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_network_usage memory_usage(::robot_interfaces::msg::RobotState::_memory_usage_type arg)
  {
    msg_.memory_usage = std::move(arg);
    return Init_RobotState_network_usage(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_cpu_usage
{
public:
  explicit Init_RobotState_cpu_usage(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_memory_usage cpu_usage(::robot_interfaces::msg::RobotState::_cpu_usage_type arg)
  {
    msg_.cpu_usage = std::move(arg);
    return Init_RobotState_memory_usage(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_warnings
{
public:
  explicit Init_RobotState_warnings(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_cpu_usage warnings(::robot_interfaces::msg::RobotState::_warnings_type arg)
  {
    msg_.warnings = std::move(arg);
    return Init_RobotState_cpu_usage(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_active_errors
{
public:
  explicit Init_RobotState_active_errors(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_warnings active_errors(::robot_interfaces::msg::RobotState::_active_errors_type arg)
  {
    msg_.active_errors = std::move(arg);
    return Init_RobotState_warnings(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_can_perceive
{
public:
  explicit Init_RobotState_can_perceive(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_active_errors can_perceive(::robot_interfaces::msg::RobotState::_can_perceive_type arg)
  {
    msg_.can_perceive = std::move(arg);
    return Init_RobotState_active_errors(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_can_navigate
{
public:
  explicit Init_RobotState_can_navigate(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_can_perceive can_navigate(::robot_interfaces::msg::RobotState::_can_navigate_type arg)
  {
    msg_.can_navigate = std::move(arg);
    return Init_RobotState_can_perceive(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_can_move
{
public:
  explicit Init_RobotState_can_move(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_can_navigate can_move(::robot_interfaces::msg::RobotState::_can_move_type arg)
  {
    msg_.can_move = std::move(arg);
    return Init_RobotState_can_navigate(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_sensors_ok
{
public:
  explicit Init_RobotState_sensors_ok(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_can_move sensors_ok(::robot_interfaces::msg::RobotState::_sensors_ok_type arg)
  {
    msg_.sensors_ok = std::move(arg);
    return Init_RobotState_can_move(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_communication_ok
{
public:
  explicit Init_RobotState_communication_ok(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_sensors_ok communication_ok(::robot_interfaces::msg::RobotState::_communication_ok_type arg)
  {
    msg_.communication_ok = std::move(arg);
    return Init_RobotState_sensors_ok(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_software_ok
{
public:
  explicit Init_RobotState_software_ok(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_communication_ok software_ok(::robot_interfaces::msg::RobotState::_software_ok_type arg)
  {
    msg_.software_ok = std::move(arg);
    return Init_RobotState_communication_ok(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_hardware_ok
{
public:
  explicit Init_RobotState_hardware_ok(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_software_ok hardware_ok(::robot_interfaces::msg::RobotState::_hardware_ok_type arg)
  {
    msg_.hardware_ok = std::move(arg);
    return Init_RobotState_software_ok(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_state_change_time
{
public:
  explicit Init_RobotState_state_change_time(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_hardware_ok state_change_time(::robot_interfaces::msg::RobotState::_state_change_time_type arg)
  {
    msg_.state_change_time = std::move(arg);
    return Init_RobotState_hardware_ok(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_previous_state
{
public:
  explicit Init_RobotState_previous_state(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_state_change_time previous_state(::robot_interfaces::msg::RobotState::_previous_state_type arg)
  {
    msg_.previous_state = std::move(arg);
    return Init_RobotState_state_change_time(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_current_state
{
public:
  explicit Init_RobotState_current_state(::robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_previous_state current_state(::robot_interfaces::msg::RobotState::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_RobotState_previous_state(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_header
{
public:
  Init_RobotState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_current_state header(::robot_interfaces::msg::RobotState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotState_current_state(msg_);
  }

private:
  ::robot_interfaces::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::RobotState>()
{
  return robot_interfaces::msg::builder::Init_RobotState_header();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
