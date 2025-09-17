// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:action/Navigation.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__ACTION__DETAIL__NAVIGATION__TRAITS_HPP_
#define ROBOT_INTERFACES__ACTION__DETAIL__NAVIGATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/action/detail/navigation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Navigation_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_pose
  {
    out << "target_pose: ";
    to_flow_style_yaml(msg.target_pose, out);
    out << ", ";
  }

  // member: navigation_mode
  {
    out << "navigation_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.navigation_mode, out);
    out << ", ";
  }

  // member: max_velocity
  {
    out << "max_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.max_velocity, out);
    out << ", ";
  }

  // member: tolerance
  {
    out << "tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigation_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pose:\n";
    to_block_style_yaml(msg.target_pose, out, indentation + 2);
  }

  // member: navigation_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "navigation_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.navigation_mode, out);
    out << "\n";
  }

  // member: max_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.max_velocity, out);
    out << "\n";
  }

  // member: tolerance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigation_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::action::Navigation_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::action::Navigation_Goal & msg)
{
  return robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::action::Navigation_Goal>()
{
  return "robot_interfaces::action::Navigation_Goal";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_Goal>()
{
  return "robot_interfaces/action/Navigation_Goal";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::action::Navigation_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'final_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'time_elapsed'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Navigation_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: result_message
  {
    out << "result_message: ";
    rosidl_generator_traits::value_to_yaml(msg.result_message, out);
    out << ", ";
  }

  // member: final_pose
  {
    out << "final_pose: ";
    to_flow_style_yaml(msg.final_pose, out);
    out << ", ";
  }

  // member: distance_traveled
  {
    out << "distance_traveled: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_traveled, out);
    out << ", ";
  }

  // member: time_elapsed
  {
    out << "time_elapsed: ";
    to_flow_style_yaml(msg.time_elapsed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigation_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: result_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result_message: ";
    rosidl_generator_traits::value_to_yaml(msg.result_message, out);
    out << "\n";
  }

  // member: final_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_pose:\n";
    to_block_style_yaml(msg.final_pose, out, indentation + 2);
  }

  // member: distance_traveled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_traveled: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_traveled, out);
    out << "\n";
  }

  // member: time_elapsed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_elapsed:\n";
    to_block_style_yaml(msg.time_elapsed, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigation_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::action::Navigation_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::action::Navigation_Result & msg)
{
  return robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::action::Navigation_Result>()
{
  return "robot_interfaces::action::Navigation_Result";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_Result>()
{
  return "robot_interfaces/action/Navigation_Result";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::action::Navigation_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Navigation_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_pose
  {
    out << "current_pose: ";
    to_flow_style_yaml(msg.current_pose, out);
    out << ", ";
  }

  // member: distance_remaining
  {
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
    out << ", ";
  }

  // member: estimated_time_remaining
  {
    out << "estimated_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_remaining, out);
    out << ", ";
  }

  // member: current_status
  {
    out << "current_status: ";
    rosidl_generator_traits::value_to_yaml(msg.current_status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigation_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_pose:\n";
    to_block_style_yaml(msg.current_pose, out, indentation + 2);
  }

  // member: distance_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
    out << "\n";
  }

  // member: estimated_time_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_remaining, out);
    out << "\n";
  }

  // member: current_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_status: ";
    rosidl_generator_traits::value_to_yaml(msg.current_status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigation_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::action::Navigation_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::action::Navigation_Feedback & msg)
{
  return robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::action::Navigation_Feedback>()
{
  return "robot_interfaces::action::Navigation_Feedback";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_Feedback>()
{
  return "robot_interfaces/action/Navigation_Feedback";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::action::Navigation_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "robot_interfaces/action/detail/navigation__traits.hpp"

namespace robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Navigation_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigation_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigation_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::action::Navigation_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::action::Navigation_SendGoal_Request & msg)
{
  return robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::action::Navigation_SendGoal_Request>()
{
  return "robot_interfaces::action::Navigation_SendGoal_Request";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_SendGoal_Request>()
{
  return "robot_interfaces/action/Navigation_SendGoal_Request";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<robot_interfaces::action::Navigation_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<robot_interfaces::action::Navigation_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<robot_interfaces::action::Navigation_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Navigation_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigation_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigation_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::action::Navigation_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::action::Navigation_SendGoal_Response & msg)
{
  return robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::action::Navigation_SendGoal_Response>()
{
  return "robot_interfaces::action::Navigation_SendGoal_Response";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_SendGoal_Response>()
{
  return "robot_interfaces/action/Navigation_SendGoal_Response";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<robot_interfaces::action::Navigation_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_interfaces::action::Navigation_SendGoal>()
{
  return "robot_interfaces::action::Navigation_SendGoal";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_SendGoal>()
{
  return "robot_interfaces/action/Navigation_SendGoal";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_interfaces::action::Navigation_SendGoal_Request>::value &&
    has_fixed_size<robot_interfaces::action::Navigation_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_interfaces::action::Navigation_SendGoal_Request>::value &&
    has_bounded_size<robot_interfaces::action::Navigation_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<robot_interfaces::action::Navigation_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<robot_interfaces::action::Navigation_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_interfaces::action::Navigation_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Navigation_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigation_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigation_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::action::Navigation_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::action::Navigation_GetResult_Request & msg)
{
  return robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::action::Navigation_GetResult_Request>()
{
  return "robot_interfaces::action::Navigation_GetResult_Request";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_GetResult_Request>()
{
  return "robot_interfaces/action/Navigation_GetResult_Request";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<robot_interfaces::action::Navigation_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "robot_interfaces/action/detail/navigation__traits.hpp"

namespace robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Navigation_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigation_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigation_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::action::Navigation_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::action::Navigation_GetResult_Response & msg)
{
  return robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::action::Navigation_GetResult_Response>()
{
  return "robot_interfaces::action::Navigation_GetResult_Response";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_GetResult_Response>()
{
  return "robot_interfaces/action/Navigation_GetResult_Response";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<robot_interfaces::action::Navigation_Result>::value> {};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<robot_interfaces::action::Navigation_Result>::value> {};

template<>
struct is_message<robot_interfaces::action::Navigation_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_interfaces::action::Navigation_GetResult>()
{
  return "robot_interfaces::action::Navigation_GetResult";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_GetResult>()
{
  return "robot_interfaces/action/Navigation_GetResult";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_interfaces::action::Navigation_GetResult_Request>::value &&
    has_fixed_size<robot_interfaces::action::Navigation_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_interfaces::action::Navigation_GetResult_Request>::value &&
    has_bounded_size<robot_interfaces::action::Navigation_GetResult_Response>::value
  >
{
};

template<>
struct is_service<robot_interfaces::action::Navigation_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<robot_interfaces::action::Navigation_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_interfaces::action::Navigation_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "robot_interfaces/action/detail/navigation__traits.hpp"

namespace robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Navigation_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigation_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigation_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::action::Navigation_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::action::Navigation_FeedbackMessage & msg)
{
  return robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::action::Navigation_FeedbackMessage>()
{
  return "robot_interfaces::action::Navigation_FeedbackMessage";
}

template<>
inline const char * name<robot_interfaces::action::Navigation_FeedbackMessage>()
{
  return "robot_interfaces/action/Navigation_FeedbackMessage";
}

template<>
struct has_fixed_size<robot_interfaces::action::Navigation_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<robot_interfaces::action::Navigation_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<robot_interfaces::action::Navigation_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<robot_interfaces::action::Navigation_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<robot_interfaces::action::Navigation_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<robot_interfaces::action::Navigation>
  : std::true_type
{
};

template<>
struct is_action_goal<robot_interfaces::action::Navigation_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<robot_interfaces::action::Navigation_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<robot_interfaces::action::Navigation_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ROBOT_INTERFACES__ACTION__DETAIL__NAVIGATION__TRAITS_HPP_
