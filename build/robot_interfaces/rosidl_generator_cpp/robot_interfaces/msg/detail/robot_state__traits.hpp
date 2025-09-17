// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'state_change_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: current_state
  {
    out << "current_state: ";
    rosidl_generator_traits::value_to_yaml(msg.current_state, out);
    out << ", ";
  }

  // member: previous_state
  {
    out << "previous_state: ";
    rosidl_generator_traits::value_to_yaml(msg.previous_state, out);
    out << ", ";
  }

  // member: state_change_time
  {
    out << "state_change_time: ";
    to_flow_style_yaml(msg.state_change_time, out);
    out << ", ";
  }

  // member: hardware_ok
  {
    out << "hardware_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_ok, out);
    out << ", ";
  }

  // member: software_ok
  {
    out << "software_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.software_ok, out);
    out << ", ";
  }

  // member: communication_ok
  {
    out << "communication_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_ok, out);
    out << ", ";
  }

  // member: sensors_ok
  {
    out << "sensors_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_ok, out);
    out << ", ";
  }

  // member: can_move
  {
    out << "can_move: ";
    rosidl_generator_traits::value_to_yaml(msg.can_move, out);
    out << ", ";
  }

  // member: can_navigate
  {
    out << "can_navigate: ";
    rosidl_generator_traits::value_to_yaml(msg.can_navigate, out);
    out << ", ";
  }

  // member: can_perceive
  {
    out << "can_perceive: ";
    rosidl_generator_traits::value_to_yaml(msg.can_perceive, out);
    out << ", ";
  }

  // member: active_errors
  {
    if (msg.active_errors.size() == 0) {
      out << "active_errors: []";
    } else {
      out << "active_errors: [";
      size_t pending_items = msg.active_errors.size();
      for (auto item : msg.active_errors) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: warnings
  {
    if (msg.warnings.size() == 0) {
      out << "warnings: []";
    } else {
      out << "warnings: [";
      size_t pending_items = msg.warnings.size();
      for (auto item : msg.warnings) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cpu_usage
  {
    out << "cpu_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_usage, out);
    out << ", ";
  }

  // member: memory_usage
  {
    out << "memory_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.memory_usage, out);
    out << ", ";
  }

  // member: network_usage
  {
    out << "network_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.network_usage, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: current_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_state: ";
    rosidl_generator_traits::value_to_yaml(msg.current_state, out);
    out << "\n";
  }

  // member: previous_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "previous_state: ";
    rosidl_generator_traits::value_to_yaml(msg.previous_state, out);
    out << "\n";
  }

  // member: state_change_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_change_time:\n";
    to_block_style_yaml(msg.state_change_time, out, indentation + 2);
  }

  // member: hardware_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hardware_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_ok, out);
    out << "\n";
  }

  // member: software_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "software_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.software_ok, out);
    out << "\n";
  }

  // member: communication_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_ok, out);
    out << "\n";
  }

  // member: sensors_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensors_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_ok, out);
    out << "\n";
  }

  // member: can_move
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "can_move: ";
    rosidl_generator_traits::value_to_yaml(msg.can_move, out);
    out << "\n";
  }

  // member: can_navigate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "can_navigate: ";
    rosidl_generator_traits::value_to_yaml(msg.can_navigate, out);
    out << "\n";
  }

  // member: can_perceive
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "can_perceive: ";
    rosidl_generator_traits::value_to_yaml(msg.can_perceive, out);
    out << "\n";
  }

  // member: active_errors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_errors.size() == 0) {
      out << "active_errors: []\n";
    } else {
      out << "active_errors:\n";
      for (auto item : msg.active_errors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: warnings
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.warnings.size() == 0) {
      out << "warnings: []\n";
    } else {
      out << "warnings:\n";
      for (auto item : msg.warnings) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cpu_usage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpu_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_usage, out);
    out << "\n";
  }

  // member: memory_usage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "memory_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.memory_usage, out);
    out << "\n";
  }

  // member: network_usage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "network_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.network_usage, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::msg::RobotState & msg)
{
  return robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::msg::RobotState>()
{
  return "robot_interfaces::msg::RobotState";
}

template<>
inline const char * name<robot_interfaces::msg::RobotState>()
{
  return "robot_interfaces/msg/RobotState";
}

template<>
struct has_fixed_size<robot_interfaces::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
