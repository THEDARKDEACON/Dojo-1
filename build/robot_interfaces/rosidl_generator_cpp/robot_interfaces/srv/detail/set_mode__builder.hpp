// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:srv/SetMode.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__SRV__DETAIL__SET_MODE__BUILDER_HPP_
#define ROBOT_INTERFACES__SRV__DETAIL__SET_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/srv/detail/set_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetMode_Request_parameters
{
public:
  explicit Init_SetMode_Request_parameters(::robot_interfaces::srv::SetMode_Request & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::srv::SetMode_Request parameters(::robot_interfaces::srv::SetMode_Request::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::srv::SetMode_Request msg_;
};

class Init_SetMode_Request_requested_mode
{
public:
  Init_SetMode_Request_requested_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Request_parameters requested_mode(::robot_interfaces::srv::SetMode_Request::_requested_mode_type arg)
  {
    msg_.requested_mode = std::move(arg);
    return Init_SetMode_Request_parameters(msg_);
  }

private:
  ::robot_interfaces::srv::SetMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::srv::SetMode_Request>()
{
  return robot_interfaces::srv::builder::Init_SetMode_Request_requested_mode();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetMode_Response_message
{
public:
  explicit Init_SetMode_Response_message(::robot_interfaces::srv::SetMode_Response & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::srv::SetMode_Response message(::robot_interfaces::srv::SetMode_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::srv::SetMode_Response msg_;
};

class Init_SetMode_Response_current_mode
{
public:
  explicit Init_SetMode_Response_current_mode(::robot_interfaces::srv::SetMode_Response & msg)
  : msg_(msg)
  {}
  Init_SetMode_Response_message current_mode(::robot_interfaces::srv::SetMode_Response::_current_mode_type arg)
  {
    msg_.current_mode = std::move(arg);
    return Init_SetMode_Response_message(msg_);
  }

private:
  ::robot_interfaces::srv::SetMode_Response msg_;
};

class Init_SetMode_Response_success
{
public:
  Init_SetMode_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Response_current_mode success(::robot_interfaces::srv::SetMode_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetMode_Response_current_mode(msg_);
  }

private:
  ::robot_interfaces::srv::SetMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::srv::SetMode_Response>()
{
  return robot_interfaces::srv::builder::Init_SetMode_Response_success();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__SRV__DETAIL__SET_MODE__BUILDER_HPP_
