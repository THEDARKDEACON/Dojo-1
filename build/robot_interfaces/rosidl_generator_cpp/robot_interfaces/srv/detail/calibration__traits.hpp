// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:srv/Calibration.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__SRV__DETAIL__CALIBRATION__TRAITS_HPP_
#define ROBOT_INTERFACES__SRV__DETAIL__CALIBRATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/srv/detail/calibration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Calibration_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: component_name
  {
    out << "component_name: ";
    rosidl_generator_traits::value_to_yaml(msg.component_name, out);
    out << ", ";
  }

  // member: calibration_type
  {
    out << "calibration_type: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_type, out);
    out << ", ";
  }

  // member: parameters
  {
    if (msg.parameters.size() == 0) {
      out << "parameters: []";
    } else {
      out << "parameters: [";
      size_t pending_items = msg.parameters.size();
      for (auto item : msg.parameters) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: values
  {
    if (msg.values.size() == 0) {
      out << "values: []";
    } else {
      out << "values: [";
      size_t pending_items = msg.values.size();
      for (auto item : msg.values) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Calibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: component_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "component_name: ";
    rosidl_generator_traits::value_to_yaml(msg.component_name, out);
    out << "\n";
  }

  // member: calibration_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_type: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_type, out);
    out << "\n";
  }

  // member: parameters
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.parameters.size() == 0) {
      out << "parameters: []\n";
    } else {
      out << "parameters:\n";
      for (auto item : msg.parameters) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.values.size() == 0) {
      out << "values: []\n";
    } else {
      out << "values:\n";
      for (auto item : msg.values) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Calibration_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::srv::Calibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::srv::Calibration_Request & msg)
{
  return robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::srv::Calibration_Request>()
{
  return "robot_interfaces::srv::Calibration_Request";
}

template<>
inline const char * name<robot_interfaces::srv::Calibration_Request>()
{
  return "robot_interfaces/srv/Calibration_Request";
}

template<>
struct has_fixed_size<robot_interfaces::srv::Calibration_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::srv::Calibration_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::srv::Calibration_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Calibration_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: calibration_results
  {
    if (msg.calibration_results.size() == 0) {
      out << "calibration_results: []";
    } else {
      out << "calibration_results: [";
      size_t pending_items = msg.calibration_results.size();
      for (auto item : msg.calibration_results) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Calibration_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: calibration_results
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.calibration_results.size() == 0) {
      out << "calibration_results: []\n";
    } else {
      out << "calibration_results:\n";
      for (auto item : msg.calibration_results) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Calibration_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::srv::Calibration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::srv::Calibration_Response & msg)
{
  return robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::srv::Calibration_Response>()
{
  return "robot_interfaces::srv::Calibration_Response";
}

template<>
inline const char * name<robot_interfaces::srv::Calibration_Response>()
{
  return "robot_interfaces/srv/Calibration_Response";
}

template<>
struct has_fixed_size<robot_interfaces::srv::Calibration_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::srv::Calibration_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::srv::Calibration_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_interfaces::srv::Calibration>()
{
  return "robot_interfaces::srv::Calibration";
}

template<>
inline const char * name<robot_interfaces::srv::Calibration>()
{
  return "robot_interfaces/srv/Calibration";
}

template<>
struct has_fixed_size<robot_interfaces::srv::Calibration>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_interfaces::srv::Calibration_Request>::value &&
    has_fixed_size<robot_interfaces::srv::Calibration_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_interfaces::srv::Calibration>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_interfaces::srv::Calibration_Request>::value &&
    has_bounded_size<robot_interfaces::srv::Calibration_Response>::value
  >
{
};

template<>
struct is_service<robot_interfaces::srv::Calibration>
  : std::true_type
{
};

template<>
struct is_service_request<robot_interfaces::srv::Calibration_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_interfaces::srv::Calibration_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__SRV__DETAIL__CALIBRATION__TRAITS_HPP_
