// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/SensorData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__SENSOR_DATA__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__SENSOR_DATA__STRUCT_HPP_

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
// Member 'linear_acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__SensorData __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__SensorData __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorData_
{
  using Type = SensorData_<ContainerAllocator>;

  explicit SensorData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    linear_acceleration(_init),
    angular_velocity(_init),
    orientation(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_encoder = 0l;
      this->right_encoder = 0l;
      this->left_velocity = 0.0;
      this->right_velocity = 0.0;
      this->ultrasonic_distance = 0.0;
      this->ultrasonic_valid = false;
      this->imu_available = false;
      this->battery_voltage = 0.0;
      this->current_draw = 0.0;
    }
  }

  explicit SensorData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    linear_acceleration(_alloc, _init),
    angular_velocity(_alloc, _init),
    orientation(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_encoder = 0l;
      this->right_encoder = 0l;
      this->left_velocity = 0.0;
      this->right_velocity = 0.0;
      this->ultrasonic_distance = 0.0;
      this->ultrasonic_valid = false;
      this->imu_available = false;
      this->battery_voltage = 0.0;
      this->current_draw = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _left_encoder_type =
    int32_t;
  _left_encoder_type left_encoder;
  using _right_encoder_type =
    int32_t;
  _right_encoder_type right_encoder;
  using _left_velocity_type =
    double;
  _left_velocity_type left_velocity;
  using _right_velocity_type =
    double;
  _right_velocity_type right_velocity;
  using _ultrasonic_distance_type =
    double;
  _ultrasonic_distance_type ultrasonic_distance;
  using _ultrasonic_valid_type =
    bool;
  _ultrasonic_valid_type ultrasonic_valid;
  using _imu_available_type =
    bool;
  _imu_available_type imu_available;
  using _linear_acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _linear_acceleration_type linear_acceleration;
  using _angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_velocity_type angular_velocity;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _battery_voltage_type =
    double;
  _battery_voltage_type battery_voltage;
  using _current_draw_type =
    double;
  _current_draw_type current_draw;
  using _temperatures_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _temperatures_type temperatures;
  using _temperature_labels_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _temperature_labels_type temperature_labels;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__left_encoder(
    const int32_t & _arg)
  {
    this->left_encoder = _arg;
    return *this;
  }
  Type & set__right_encoder(
    const int32_t & _arg)
  {
    this->right_encoder = _arg;
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
  Type & set__ultrasonic_distance(
    const double & _arg)
  {
    this->ultrasonic_distance = _arg;
    return *this;
  }
  Type & set__ultrasonic_valid(
    const bool & _arg)
  {
    this->ultrasonic_valid = _arg;
    return *this;
  }
  Type & set__imu_available(
    const bool & _arg)
  {
    this->imu_available = _arg;
    return *this;
  }
  Type & set__linear_acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->linear_acceleration = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const double & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__current_draw(
    const double & _arg)
  {
    this->current_draw = _arg;
    return *this;
  }
  Type & set__temperatures(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->temperatures = _arg;
    return *this;
  }
  Type & set__temperature_labels(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->temperature_labels = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::SensorData_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::SensorData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::SensorData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::SensorData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::SensorData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::SensorData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::SensorData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::SensorData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::SensorData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::SensorData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__SensorData
    std::shared_ptr<robot_interfaces::msg::SensorData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__SensorData
    std::shared_ptr<robot_interfaces::msg::SensorData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->left_encoder != other.left_encoder) {
      return false;
    }
    if (this->right_encoder != other.right_encoder) {
      return false;
    }
    if (this->left_velocity != other.left_velocity) {
      return false;
    }
    if (this->right_velocity != other.right_velocity) {
      return false;
    }
    if (this->ultrasonic_distance != other.ultrasonic_distance) {
      return false;
    }
    if (this->ultrasonic_valid != other.ultrasonic_valid) {
      return false;
    }
    if (this->imu_available != other.imu_available) {
      return false;
    }
    if (this->linear_acceleration != other.linear_acceleration) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->current_draw != other.current_draw) {
      return false;
    }
    if (this->temperatures != other.temperatures) {
      return false;
    }
    if (this->temperature_labels != other.temperature_labels) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorData_

// alias to use template instance with default allocator
using SensorData =
  robot_interfaces::msg::SensorData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__SENSOR_DATA__STRUCT_HPP_
