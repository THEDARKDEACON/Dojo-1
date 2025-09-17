// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__MotorCommand __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__MotorCommand __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCommand_
{
  using Type = MotorCommand_<ContainerAllocator>;

  explicit MotorCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_mode = 0;
      this->left_velocity = 0.0;
      this->right_velocity = 0.0;
      this->left_pwm = 0;
      this->right_pwm = 0;
      this->left_position = 0.0;
      this->right_position = 0.0;
      this->max_velocity = 0.0;
      this->max_acceleration = 0.0;
      this->emergency_stop = false;
    }
  }

  explicit MotorCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_mode = 0;
      this->left_velocity = 0.0;
      this->right_velocity = 0.0;
      this->left_pwm = 0;
      this->right_pwm = 0;
      this->left_position = 0.0;
      this->right_position = 0.0;
      this->max_velocity = 0.0;
      this->max_acceleration = 0.0;
      this->emergency_stop = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _control_mode_type =
    uint8_t;
  _control_mode_type control_mode;
  using _left_velocity_type =
    double;
  _left_velocity_type left_velocity;
  using _right_velocity_type =
    double;
  _right_velocity_type right_velocity;
  using _left_pwm_type =
    int16_t;
  _left_pwm_type left_pwm;
  using _right_pwm_type =
    int16_t;
  _right_pwm_type right_pwm;
  using _left_position_type =
    double;
  _left_position_type left_position;
  using _right_position_type =
    double;
  _right_position_type right_position;
  using _max_velocity_type =
    double;
  _max_velocity_type max_velocity;
  using _max_acceleration_type =
    double;
  _max_acceleration_type max_acceleration;
  using _emergency_stop_type =
    bool;
  _emergency_stop_type emergency_stop;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__control_mode(
    const uint8_t & _arg)
  {
    this->control_mode = _arg;
    return *this;
  }
  Type & set__left_velocity(
    const double & _arg)
  {
    this->left_velocity = _arg;
    return *this;
  }
  Type & set__right_velocity(
    const double & _arg)
  {
    this->right_velocity = _arg;
    return *this;
  }
  Type & set__left_pwm(
    const int16_t & _arg)
  {
    this->left_pwm = _arg;
    return *this;
  }
  Type & set__right_pwm(
    const int16_t & _arg)
  {
    this->right_pwm = _arg;
    return *this;
  }
  Type & set__left_position(
    const double & _arg)
  {
    this->left_position = _arg;
    return *this;
  }
  Type & set__right_position(
    const double & _arg)
  {
    this->right_position = _arg;
    return *this;
  }
  Type & set__max_velocity(
    const double & _arg)
  {
    this->max_velocity = _arg;
    return *this;
  }
  Type & set__max_acceleration(
    const double & _arg)
  {
    this->max_acceleration = _arg;
    return *this;
  }
  Type & set__emergency_stop(
    const bool & _arg)
  {
    this->emergency_stop = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t VELOCITY_MODE =
    0u;
  static constexpr uint8_t PWM_MODE =
    1u;
  static constexpr uint8_t POSITION_MODE =
    2u;

  // pointer types
  using RawPtr =
    robot_interfaces::msg::MotorCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::MotorCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::MotorCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::MotorCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::MotorCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::MotorCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::MotorCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::MotorCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::MotorCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::MotorCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__MotorCommand
    std::shared_ptr<robot_interfaces::msg::MotorCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__MotorCommand
    std::shared_ptr<robot_interfaces::msg::MotorCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->control_mode != other.control_mode) {
      return false;
    }
    if (this->left_velocity != other.left_velocity) {
      return false;
    }
    if (this->right_velocity != other.right_velocity) {
      return false;
    }
    if (this->left_pwm != other.left_pwm) {
      return false;
    }
    if (this->right_pwm != other.right_pwm) {
      return false;
    }
    if (this->left_position != other.left_position) {
      return false;
    }
    if (this->right_position != other.right_position) {
      return false;
    }
    if (this->max_velocity != other.max_velocity) {
      return false;
    }
    if (this->max_acceleration != other.max_acceleration) {
      return false;
    }
    if (this->emergency_stop != other.emergency_stop) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommand_

// alias to use template instance with default allocator
using MotorCommand =
  robot_interfaces::msg::MotorCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MotorCommand_<ContainerAllocator>::VELOCITY_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MotorCommand_<ContainerAllocator>::PWM_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MotorCommand_<ContainerAllocator>::POSITION_MODE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
