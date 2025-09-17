// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "robot_interfaces/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state_change_time`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `active_errors`
// Member `warnings`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_interfaces__msg__RobotState__init(robot_interfaces__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    robot_interfaces__msg__RobotState__fini(msg);
    return false;
  }
  // current_state
  // previous_state
  // state_change_time
  if (!builtin_interfaces__msg__Time__init(&msg->state_change_time)) {
    robot_interfaces__msg__RobotState__fini(msg);
    return false;
  }
  // hardware_ok
  // software_ok
  // communication_ok
  // sensors_ok
  // can_move
  // can_navigate
  // can_perceive
  // active_errors
  if (!rosidl_runtime_c__String__Sequence__init(&msg->active_errors, 0)) {
    robot_interfaces__msg__RobotState__fini(msg);
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__Sequence__init(&msg->warnings, 0)) {
    robot_interfaces__msg__RobotState__fini(msg);
    return false;
  }
  // cpu_usage
  // memory_usage
  // network_usage
  return true;
}

void
robot_interfaces__msg__RobotState__fini(robot_interfaces__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // current_state
  // previous_state
  // state_change_time
  builtin_interfaces__msg__Time__fini(&msg->state_change_time);
  // hardware_ok
  // software_ok
  // communication_ok
  // sensors_ok
  // can_move
  // can_navigate
  // can_perceive
  // active_errors
  rosidl_runtime_c__String__Sequence__fini(&msg->active_errors);
  // warnings
  rosidl_runtime_c__String__Sequence__fini(&msg->warnings);
  // cpu_usage
  // memory_usage
  // network_usage
}

bool
robot_interfaces__msg__RobotState__are_equal(const robot_interfaces__msg__RobotState * lhs, const robot_interfaces__msg__RobotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // current_state
  if (lhs->current_state != rhs->current_state) {
    return false;
  }
  // previous_state
  if (lhs->previous_state != rhs->previous_state) {
    return false;
  }
  // state_change_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->state_change_time), &(rhs->state_change_time)))
  {
    return false;
  }
  // hardware_ok
  if (lhs->hardware_ok != rhs->hardware_ok) {
    return false;
  }
  // software_ok
  if (lhs->software_ok != rhs->software_ok) {
    return false;
  }
  // communication_ok
  if (lhs->communication_ok != rhs->communication_ok) {
    return false;
  }
  // sensors_ok
  if (lhs->sensors_ok != rhs->sensors_ok) {
    return false;
  }
  // can_move
  if (lhs->can_move != rhs->can_move) {
    return false;
  }
  // can_navigate
  if (lhs->can_navigate != rhs->can_navigate) {
    return false;
  }
  // can_perceive
  if (lhs->can_perceive != rhs->can_perceive) {
    return false;
  }
  // active_errors
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->active_errors), &(rhs->active_errors)))
  {
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->warnings), &(rhs->warnings)))
  {
    return false;
  }
  // cpu_usage
  if (lhs->cpu_usage != rhs->cpu_usage) {
    return false;
  }
  // memory_usage
  if (lhs->memory_usage != rhs->memory_usage) {
    return false;
  }
  // network_usage
  if (lhs->network_usage != rhs->network_usage) {
    return false;
  }
  return true;
}

bool
robot_interfaces__msg__RobotState__copy(
  const robot_interfaces__msg__RobotState * input,
  robot_interfaces__msg__RobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // current_state
  output->current_state = input->current_state;
  // previous_state
  output->previous_state = input->previous_state;
  // state_change_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->state_change_time), &(output->state_change_time)))
  {
    return false;
  }
  // hardware_ok
  output->hardware_ok = input->hardware_ok;
  // software_ok
  output->software_ok = input->software_ok;
  // communication_ok
  output->communication_ok = input->communication_ok;
  // sensors_ok
  output->sensors_ok = input->sensors_ok;
  // can_move
  output->can_move = input->can_move;
  // can_navigate
  output->can_navigate = input->can_navigate;
  // can_perceive
  output->can_perceive = input->can_perceive;
  // active_errors
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->active_errors), &(output->active_errors)))
  {
    return false;
  }
  // warnings
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->warnings), &(output->warnings)))
  {
    return false;
  }
  // cpu_usage
  output->cpu_usage = input->cpu_usage;
  // memory_usage
  output->memory_usage = input->memory_usage;
  // network_usage
  output->network_usage = input->network_usage;
  return true;
}

robot_interfaces__msg__RobotState *
robot_interfaces__msg__RobotState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__RobotState * msg = (robot_interfaces__msg__RobotState *)allocator.allocate(sizeof(robot_interfaces__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_interfaces__msg__RobotState));
  bool success = robot_interfaces__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_interfaces__msg__RobotState__destroy(robot_interfaces__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_interfaces__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_interfaces__msg__RobotState__Sequence__init(robot_interfaces__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__RobotState * data = NULL;

  if (size) {
    data = (robot_interfaces__msg__RobotState *)allocator.zero_allocate(size, sizeof(robot_interfaces__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_interfaces__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_interfaces__msg__RobotState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_interfaces__msg__RobotState__Sequence__fini(robot_interfaces__msg__RobotState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_interfaces__msg__RobotState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_interfaces__msg__RobotState__Sequence *
robot_interfaces__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__RobotState__Sequence * array = (robot_interfaces__msg__RobotState__Sequence *)allocator.allocate(sizeof(robot_interfaces__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_interfaces__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_interfaces__msg__RobotState__Sequence__destroy(robot_interfaces__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_interfaces__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_interfaces__msg__RobotState__Sequence__are_equal(const robot_interfaces__msg__RobotState__Sequence * lhs, const robot_interfaces__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_interfaces__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_interfaces__msg__RobotState__Sequence__copy(
  const robot_interfaces__msg__RobotState__Sequence * input,
  robot_interfaces__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_interfaces__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_interfaces__msg__RobotState * data =
      (robot_interfaces__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_interfaces__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_interfaces__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_interfaces__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
