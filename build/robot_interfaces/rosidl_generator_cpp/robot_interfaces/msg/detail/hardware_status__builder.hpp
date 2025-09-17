// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/HardwareStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__HARDWARE_STATUS__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__HARDWARE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/hardware_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_HardwareStatus_data_values
{
public:
  explicit Init_HardwareStatus_data_values(::robot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::HardwareStatus data_values(::robot_interfaces::msg::HardwareStatus::_data_values_type arg)
  {
    msg_.data_values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_data_keys
{
public:
  explicit Init_HardwareStatus_data_keys(::robot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  Init_HardwareStatus_data_values data_keys(::robot_interfaces::msg::HardwareStatus::_data_keys_type arg)
  {
    msg_.data_keys = std::move(arg);
    return Init_HardwareStatus_data_values(msg_);
  }

private:
  ::robot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_last_update
{
public:
  explicit Init_HardwareStatus_last_update(::robot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  Init_HardwareStatus_data_keys last_update(::robot_interfaces::msg::HardwareStatus::_last_update_type arg)
  {
    msg_.last_update = std::move(arg);
    return Init_HardwareStatus_data_keys(msg_);
  }

private:
  ::robot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_uptime
{
public:
  explicit Init_HardwareStatus_uptime(::robot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  Init_HardwareStatus_last_update uptime(::robot_interfaces::msg::HardwareStatus::_uptime_type arg)
  {
    msg_.uptime = std::move(arg);
    return Init_HardwareStatus_last_update(msg_);
  }

private:
  ::robot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_status_message
{
public:
  explicit Init_HardwareStatus_status_message(::robot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  Init_HardwareStatus_uptime status_message(::robot_interfaces::msg::HardwareStatus::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return Init_HardwareStatus_uptime(msg_);
  }

private:
  ::robot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_status
{
public:
  explicit Init_HardwareStatus_status(::robot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  Init_HardwareStatus_status_message status(::robot_interfaces::msg::HardwareStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_HardwareStatus_status_message(msg_);
  }

private:
  ::robot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_hardware_id
{
public:
  explicit Init_HardwareStatus_hardware_id(::robot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  Init_HardwareStatus_status hardware_id(::robot_interfaces::msg::HardwareStatus::_hardware_id_type arg)
  {
    msg_.hardware_id = std::move(arg);
    return Init_HardwareStatus_status(msg_);
  }

private:
  ::robot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_component_name
{
public:
  explicit Init_HardwareStatus_component_name(::robot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  Init_HardwareStatus_hardware_id component_name(::robot_interfaces::msg::HardwareStatus::_component_name_type arg)
  {
    msg_.component_name = std::move(arg);
    return Init_HardwareStatus_hardware_id(msg_);
  }

private:
  ::robot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_header
{
public:
  Init_HardwareStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HardwareStatus_component_name header(::robot_interfaces::msg::HardwareStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HardwareStatus_component_name(msg_);
  }

private:
  ::robot_interfaces::msg::HardwareStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::HardwareStatus>()
{
  return robot_interfaces::msg::builder::Init_HardwareStatus_header();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__HARDWARE_STATUS__BUILDER_HPP_
