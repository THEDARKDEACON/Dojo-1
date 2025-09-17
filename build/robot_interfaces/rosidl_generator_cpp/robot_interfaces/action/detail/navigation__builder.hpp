// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:action/Navigation.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__ACTION__DETAIL__NAVIGATION__BUILDER_HPP_
#define ROBOT_INTERFACES__ACTION__DETAIL__NAVIGATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/action/detail/navigation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Navigation_Goal_tolerance
{
public:
  explicit Init_Navigation_Goal_tolerance(::robot_interfaces::action::Navigation_Goal & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Navigation_Goal tolerance(::robot_interfaces::action::Navigation_Goal::_tolerance_type arg)
  {
    msg_.tolerance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Goal msg_;
};

class Init_Navigation_Goal_max_velocity
{
public:
  explicit Init_Navigation_Goal_max_velocity(::robot_interfaces::action::Navigation_Goal & msg)
  : msg_(msg)
  {}
  Init_Navigation_Goal_tolerance max_velocity(::robot_interfaces::action::Navigation_Goal::_max_velocity_type arg)
  {
    msg_.max_velocity = std::move(arg);
    return Init_Navigation_Goal_tolerance(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Goal msg_;
};

class Init_Navigation_Goal_navigation_mode
{
public:
  explicit Init_Navigation_Goal_navigation_mode(::robot_interfaces::action::Navigation_Goal & msg)
  : msg_(msg)
  {}
  Init_Navigation_Goal_max_velocity navigation_mode(::robot_interfaces::action::Navigation_Goal::_navigation_mode_type arg)
  {
    msg_.navigation_mode = std::move(arg);
    return Init_Navigation_Goal_max_velocity(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Goal msg_;
};

class Init_Navigation_Goal_target_pose
{
public:
  Init_Navigation_Goal_target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigation_Goal_navigation_mode target_pose(::robot_interfaces::action::Navigation_Goal::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_Navigation_Goal_navigation_mode(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Navigation_Goal>()
{
  return robot_interfaces::action::builder::Init_Navigation_Goal_target_pose();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Navigation_Result_time_elapsed
{
public:
  explicit Init_Navigation_Result_time_elapsed(::robot_interfaces::action::Navigation_Result & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Navigation_Result time_elapsed(::robot_interfaces::action::Navigation_Result::_time_elapsed_type arg)
  {
    msg_.time_elapsed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Result msg_;
};

class Init_Navigation_Result_distance_traveled
{
public:
  explicit Init_Navigation_Result_distance_traveled(::robot_interfaces::action::Navigation_Result & msg)
  : msg_(msg)
  {}
  Init_Navigation_Result_time_elapsed distance_traveled(::robot_interfaces::action::Navigation_Result::_distance_traveled_type arg)
  {
    msg_.distance_traveled = std::move(arg);
    return Init_Navigation_Result_time_elapsed(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Result msg_;
};

class Init_Navigation_Result_final_pose
{
public:
  explicit Init_Navigation_Result_final_pose(::robot_interfaces::action::Navigation_Result & msg)
  : msg_(msg)
  {}
  Init_Navigation_Result_distance_traveled final_pose(::robot_interfaces::action::Navigation_Result::_final_pose_type arg)
  {
    msg_.final_pose = std::move(arg);
    return Init_Navigation_Result_distance_traveled(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Result msg_;
};

class Init_Navigation_Result_result_message
{
public:
  explicit Init_Navigation_Result_result_message(::robot_interfaces::action::Navigation_Result & msg)
  : msg_(msg)
  {}
  Init_Navigation_Result_final_pose result_message(::robot_interfaces::action::Navigation_Result::_result_message_type arg)
  {
    msg_.result_message = std::move(arg);
    return Init_Navigation_Result_final_pose(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Result msg_;
};

class Init_Navigation_Result_success
{
public:
  Init_Navigation_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigation_Result_result_message success(::robot_interfaces::action::Navigation_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Navigation_Result_result_message(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Navigation_Result>()
{
  return robot_interfaces::action::builder::Init_Navigation_Result_success();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Navigation_Feedback_current_status
{
public:
  explicit Init_Navigation_Feedback_current_status(::robot_interfaces::action::Navigation_Feedback & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Navigation_Feedback current_status(::robot_interfaces::action::Navigation_Feedback::_current_status_type arg)
  {
    msg_.current_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Feedback msg_;
};

class Init_Navigation_Feedback_estimated_time_remaining
{
public:
  explicit Init_Navigation_Feedback_estimated_time_remaining(::robot_interfaces::action::Navigation_Feedback & msg)
  : msg_(msg)
  {}
  Init_Navigation_Feedback_current_status estimated_time_remaining(::robot_interfaces::action::Navigation_Feedback::_estimated_time_remaining_type arg)
  {
    msg_.estimated_time_remaining = std::move(arg);
    return Init_Navigation_Feedback_current_status(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Feedback msg_;
};

class Init_Navigation_Feedback_distance_remaining
{
public:
  explicit Init_Navigation_Feedback_distance_remaining(::robot_interfaces::action::Navigation_Feedback & msg)
  : msg_(msg)
  {}
  Init_Navigation_Feedback_estimated_time_remaining distance_remaining(::robot_interfaces::action::Navigation_Feedback::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return Init_Navigation_Feedback_estimated_time_remaining(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Feedback msg_;
};

class Init_Navigation_Feedback_current_pose
{
public:
  Init_Navigation_Feedback_current_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigation_Feedback_distance_remaining current_pose(::robot_interfaces::action::Navigation_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_Navigation_Feedback_distance_remaining(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Navigation_Feedback>()
{
  return robot_interfaces::action::builder::Init_Navigation_Feedback_current_pose();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Navigation_SendGoal_Request_goal
{
public:
  explicit Init_Navigation_SendGoal_Request_goal(::robot_interfaces::action::Navigation_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Navigation_SendGoal_Request goal(::robot_interfaces::action::Navigation_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_SendGoal_Request msg_;
};

class Init_Navigation_SendGoal_Request_goal_id
{
public:
  Init_Navigation_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigation_SendGoal_Request_goal goal_id(::robot_interfaces::action::Navigation_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Navigation_SendGoal_Request_goal(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Navigation_SendGoal_Request>()
{
  return robot_interfaces::action::builder::Init_Navigation_SendGoal_Request_goal_id();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Navigation_SendGoal_Response_stamp
{
public:
  explicit Init_Navigation_SendGoal_Response_stamp(::robot_interfaces::action::Navigation_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Navigation_SendGoal_Response stamp(::robot_interfaces::action::Navigation_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_SendGoal_Response msg_;
};

class Init_Navigation_SendGoal_Response_accepted
{
public:
  Init_Navigation_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigation_SendGoal_Response_stamp accepted(::robot_interfaces::action::Navigation_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Navigation_SendGoal_Response_stamp(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Navigation_SendGoal_Response>()
{
  return robot_interfaces::action::builder::Init_Navigation_SendGoal_Response_accepted();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Navigation_GetResult_Request_goal_id
{
public:
  Init_Navigation_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_interfaces::action::Navigation_GetResult_Request goal_id(::robot_interfaces::action::Navigation_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Navigation_GetResult_Request>()
{
  return robot_interfaces::action::builder::Init_Navigation_GetResult_Request_goal_id();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Navigation_GetResult_Response_result
{
public:
  explicit Init_Navigation_GetResult_Response_result(::robot_interfaces::action::Navigation_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Navigation_GetResult_Response result(::robot_interfaces::action::Navigation_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_GetResult_Response msg_;
};

class Init_Navigation_GetResult_Response_status
{
public:
  Init_Navigation_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigation_GetResult_Response_result status(::robot_interfaces::action::Navigation_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Navigation_GetResult_Response_result(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Navigation_GetResult_Response>()
{
  return robot_interfaces::action::builder::Init_Navigation_GetResult_Response_status();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Navigation_FeedbackMessage_feedback
{
public:
  explicit Init_Navigation_FeedbackMessage_feedback(::robot_interfaces::action::Navigation_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Navigation_FeedbackMessage feedback(::robot_interfaces::action::Navigation_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_FeedbackMessage msg_;
};

class Init_Navigation_FeedbackMessage_goal_id
{
public:
  Init_Navigation_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigation_FeedbackMessage_feedback goal_id(::robot_interfaces::action::Navigation_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Navigation_FeedbackMessage_feedback(msg_);
  }

private:
  ::robot_interfaces::action::Navigation_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Navigation_FeedbackMessage>()
{
  return robot_interfaces::action::builder::Init_Navigation_FeedbackMessage_goal_id();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__ACTION__DETAIL__NAVIGATION__BUILDER_HPP_
