// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:srv/Calibration.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__SRV__DETAIL__CALIBRATION__BUILDER_HPP_
#define ROBOT_INTERFACES__SRV__DETAIL__CALIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/srv/detail/calibration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_Calibration_Request_values
{
public:
  explicit Init_Calibration_Request_values(::robot_interfaces::srv::Calibration_Request & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::srv::Calibration_Request values(::robot_interfaces::srv::Calibration_Request::_values_type arg)
  {
    msg_.values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::srv::Calibration_Request msg_;
};

class Init_Calibration_Request_parameters
{
public:
  explicit Init_Calibration_Request_parameters(::robot_interfaces::srv::Calibration_Request & msg)
  : msg_(msg)
  {}
  Init_Calibration_Request_values parameters(::robot_interfaces::srv::Calibration_Request::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return Init_Calibration_Request_values(msg_);
  }

private:
  ::robot_interfaces::srv::Calibration_Request msg_;
};

class Init_Calibration_Request_calibration_type
{
public:
  explicit Init_Calibration_Request_calibration_type(::robot_interfaces::srv::Calibration_Request & msg)
  : msg_(msg)
  {}
  Init_Calibration_Request_parameters calibration_type(::robot_interfaces::srv::Calibration_Request::_calibration_type_type arg)
  {
    msg_.calibration_type = std::move(arg);
    return Init_Calibration_Request_parameters(msg_);
  }

private:
  ::robot_interfaces::srv::Calibration_Request msg_;
};

class Init_Calibration_Request_component_name
{
public:
  Init_Calibration_Request_component_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Calibration_Request_calibration_type component_name(::robot_interfaces::srv::Calibration_Request::_component_name_type arg)
  {
    msg_.component_name = std::move(arg);
    return Init_Calibration_Request_calibration_type(msg_);
  }

private:
  ::robot_interfaces::srv::Calibration_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::srv::Calibration_Request>()
{
  return robot_interfaces::srv::builder::Init_Calibration_Request_component_name();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_Calibration_Response_calibration_results
{
public:
  explicit Init_Calibration_Response_calibration_results(::robot_interfaces::srv::Calibration_Response & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::srv::Calibration_Response calibration_results(::robot_interfaces::srv::Calibration_Response::_calibration_results_type arg)
  {
    msg_.calibration_results = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::srv::Calibration_Response msg_;
};

class Init_Calibration_Response_message
{
public:
  explicit Init_Calibration_Response_message(::robot_interfaces::srv::Calibration_Response & msg)
  : msg_(msg)
  {}
  Init_Calibration_Response_calibration_results message(::robot_interfaces::srv::Calibration_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_Calibration_Response_calibration_results(msg_);
  }

private:
  ::robot_interfaces::srv::Calibration_Response msg_;
};

class Init_Calibration_Response_success
{
public:
  Init_Calibration_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Calibration_Response_message success(::robot_interfaces::srv::Calibration_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Calibration_Response_message(msg_);
  }

private:
  ::robot_interfaces::srv::Calibration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::srv::Calibration_Response>()
{
  return robot_interfaces::srv::builder::Init_Calibration_Response_success();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__SRV__DETAIL__CALIBRATION__BUILDER_HPP_
