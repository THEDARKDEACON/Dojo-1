// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_

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
// Member 'state_change_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__RobotState __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__RobotState __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotState_
{
  using Type = RobotState_<ContainerAllocator>;

  explicit RobotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    state_change_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = 0;
      this->previous_state = 0;
      this->hardware_ok = false;
      this->software_ok = false;
      this->communication_ok = false;
      this->sensors_ok = false;
      this->can_move = false;
      this->can_navigate = false;
      this->can_perceive = false;
      this->cpu_usage = 0.0;
      this->memory_usage = 0.0;
      this->network_usage = 0.0;
    }
  }

  explicit RobotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state_change_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = 0;
      this->previous_state = 0;
      this->hardware_ok = false;
      this->software_ok = false;
      this->communication_ok = false;
      this->sensors_ok = false;
      this->can_move = false;
      this->can_navigate = false;
      this->can_perceive = false;
      this->cpu_usage = 0.0;
      this->memory_usage = 0.0;
      this->network_usage = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _current_state_type =
    uint8_t;
  _current_state_type current_state;
  using _previous_state_type =
    uint8_t;
  _previous_state_type previous_state;
  using _state_change_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _state_change_time_type state_change_time;
  using _hardware_ok_type =
    bool;
  _hardware_ok_type hardware_ok;
  using _software_ok_type =
    bool;
  _software_ok_type software_ok;
  using _communication_ok_type =
    bool;
  _communication_ok_type communication_ok;
  using _sensors_ok_type =
    bool;
  _sensors_ok_type sensors_ok;
  using _can_move_type =
    bool;
  _can_move_type can_move;
  using _can_navigate_type =
    bool;
  _can_navigate_type can_navigate;
  using _can_perceive_type =
    bool;
  _can_perceive_type can_perceive;
  using _active_errors_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _active_errors_type active_errors;
  using _warnings_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _warnings_type warnings;
  using _cpu_usage_type =
    double;
  _cpu_usage_type cpu_usage;
  using _memory_usage_type =
    double;
  _memory_usage_type memory_usage;
  using _network_usage_type =
    double;
  _network_usage_type network_usage;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__current_state(
    const uint8_t & _arg)
  {
    this->current_state = _arg;
    return *this;
  }
  Type & set__previous_state(
    const uint8_t & _arg)
  {
    this->previous_state = _arg;
    return *this;
  }
  Type & set__state_change_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->state_change_time = _arg;
    return *this;
  }
  Type & set__hardware_ok(
    const bool & _arg)
  {
    this->hardware_ok = _arg;
    return *this;
  }
  Type & set__software_ok(
    const bool & _arg)
  {
    this->software_ok = _arg;
    return *this;
  }
  Type & set__communication_ok(
    const bool & _arg)
  {
    this->communication_ok = _arg;
    return *this;
  }
  Type & set__sensors_ok(
    const bool & _arg)
  {
    this->sensors_ok = _arg;
    return *this;
  }
  Type & set__can_move(
    const bool & _arg)
  {
    this->can_move = _arg;
    return *this;
  }
  Type & set__can_navigate(
    const bool & _arg)
  {
    this->can_navigate = _arg;
    return *this;
  }
  Type & set__can_perceive(
    const bool & _arg)
  {
    this->can_perceive = _arg;
    return *this;
  }
  Type & set__active_errors(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->active_errors = _arg;
    return *this;
  }
  Type & set__warnings(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->warnings = _arg;
    return *this;
  }
  Type & set__cpu_usage(
    const double & _arg)
  {
    this->cpu_usage = _arg;
    return *this;
  }
  Type & set__memory_usage(
    const double & _arg)
  {
    this->memory_usage = _arg;
    return *this;
  }
  Type & set__network_usage(
    const double & _arg)
  {
    this->network_usage = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t UNKNOWN =
    0u;
  static constexpr uint8_t INITIALIZING =
    1u;
  static constexpr uint8_t IDLE =
    2u;
  static constexpr uint8_t MANUAL_CONTROL =
    3u;
  static constexpr uint8_t AUTONOMOUS =
    4u;
  // guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
  static constexpr uint8_t ERROR =
    5u;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif
  static constexpr uint8_t EMERGENCY_STOP =
    6u;
  static constexpr uint8_t SHUTDOWN =
    7u;

  // pointer types
  using RawPtr =
    robot_interfaces::msg::RobotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::RobotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::RobotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::RobotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::RobotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::RobotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::RobotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::RobotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::RobotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::RobotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__RobotState
    std::shared_ptr<robot_interfaces::msg::RobotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__RobotState
    std::shared_ptr<robot_interfaces::msg::RobotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->current_state != other.current_state) {
      return false;
    }
    if (this->previous_state != other.previous_state) {
      return false;
    }
    if (this->state_change_time != other.state_change_time) {
      return false;
    }
    if (this->hardware_ok != other.hardware_ok) {
      return false;
    }
    if (this->software_ok != other.software_ok) {
      return false;
    }
    if (this->communication_ok != other.communication_ok) {
      return false;
    }
    if (this->sensors_ok != other.sensors_ok) {
      return false;
    }
    if (this->can_move != other.can_move) {
      return false;
    }
    if (this->can_navigate != other.can_navigate) {
      return false;
    }
    if (this->can_perceive != other.can_perceive) {
      return false;
    }
    if (this->active_errors != other.active_errors) {
      return false;
    }
    if (this->warnings != other.warnings) {
      return false;
    }
    if (this->cpu_usage != other.cpu_usage) {
      return false;
    }
    if (this->memory_usage != other.memory_usage) {
      return false;
    }
    if (this->network_usage != other.network_usage) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotState_

// alias to use template instance with default allocator
using RobotState =
  robot_interfaces::msg::RobotState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobotState_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobotState_<ContainerAllocator>::INITIALIZING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobotState_<ContainerAllocator>::IDLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobotState_<ContainerAllocator>::MANUAL_CONTROL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobotState_<ContainerAllocator>::AUTONOMOUS;
#endif  // __cplusplus < 201703L
// guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobotState_<ContainerAllocator>::ERROR;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobotState_<ContainerAllocator>::EMERGENCY_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobotState_<ContainerAllocator>::SHUTDOWN;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
