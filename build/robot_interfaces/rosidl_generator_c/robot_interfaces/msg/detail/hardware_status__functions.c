// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_interfaces:msg/HardwareStatus.idl
// generated code does not contain a copyright notice
#include "robot_interfaces/msg/detail/hardware_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `component_name`
// Member `hardware_id`
// Member `status_message`
// Member `data_keys`
// Member `data_values`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_interfaces__msg__HardwareStatus__init(robot_interfaces__msg__HardwareStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    robot_interfaces__msg__HardwareStatus__fini(msg);
    return false;
  }
  // component_name
  if (!rosidl_runtime_c__String__init(&msg->component_name)) {
    robot_interfaces__msg__HardwareStatus__fini(msg);
    return false;
  }
  // hardware_id
  if (!rosidl_runtime_c__String__init(&msg->hardware_id)) {
    robot_interfaces__msg__HardwareStatus__fini(msg);
    return false;
  }
  // status
  // status_message
  if (!rosidl_runtime_c__String__init(&msg->status_message)) {
    robot_interfaces__msg__HardwareStatus__fini(msg);
    return false;
  }
  // uptime
  // last_update
  // data_keys
  if (!rosidl_runtime_c__String__Sequence__init(&msg->data_keys, 0)) {
    robot_interfaces__msg__HardwareStatus__fini(msg);
    return false;
  }
  // data_values
  if (!rosidl_runtime_c__String__Sequence__init(&msg->data_values, 0)) {
    robot_interfaces__msg__HardwareStatus__fini(msg);
    return false;
  }
  return true;
}

void
robot_interfaces__msg__HardwareStatus__fini(robot_interfaces__msg__HardwareStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // component_name
  rosidl_runtime_c__String__fini(&msg->component_name);
  // hardware_id
  rosidl_runtime_c__String__fini(&msg->hardware_id);
  // status
  // status_message
  rosidl_runtime_c__String__fini(&msg->status_message);
  // uptime
  // last_update
  // data_keys
  rosidl_runtime_c__String__Sequence__fini(&msg->data_keys);
  // data_values
  rosidl_runtime_c__String__Sequence__fini(&msg->data_values);
}

bool
robot_interfaces__msg__HardwareStatus__are_equal(const robot_interfaces__msg__HardwareStatus * lhs, const robot_interfaces__msg__HardwareStatus * rhs)
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
  // component_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->component_name), &(rhs->component_name)))
  {
    return false;
  }
  // hardware_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->hardware_id), &(rhs->hardware_id)))
  {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // status_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status_message), &(rhs->status_message)))
  {
    return false;
  }
  // uptime
  if (lhs->uptime != rhs->uptime) {
    return false;
  }
  // last_update
  if (lhs->last_update != rhs->last_update) {
    return false;
  }
  // data_keys
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->data_keys), &(rhs->data_keys)))
  {
    return false;
  }
  // data_values
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->data_values), &(rhs->data_values)))
  {
    return false;
  }
  return true;
}

bool
robot_interfaces__msg__HardwareStatus__copy(
  const robot_interfaces__msg__HardwareStatus * input,
  robot_interfaces__msg__HardwareStatus * output)
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
  // component_name
  if (!rosidl_runtime_c__String__copy(
      &(input->component_name), &(output->component_name)))
  {
    return false;
  }
  // hardware_id
  if (!rosidl_runtime_c__String__copy(
      &(input->hardware_id), &(output->hardware_id)))
  {
    return false;
  }
  // status
  output->status = input->status;
  // status_message
  if (!rosidl_runtime_c__String__copy(
      &(input->status_message), &(output->status_message)))
  {
    return false;
  }
  // uptime
  output->uptime = input->uptime;
  // last_update
  output->last_update = input->last_update;
  // data_keys
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->data_keys), &(output->data_keys)))
  {
    return false;
  }
  // data_values
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->data_values), &(output->data_values)))
  {
    return false;
  }
  return true;
}

robot_interfaces__msg__HardwareStatus *
robot_interfaces__msg__HardwareStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__HardwareStatus * msg = (robot_interfaces__msg__HardwareStatus *)allocator.allocate(sizeof(robot_interfaces__msg__HardwareStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_interfaces__msg__HardwareStatus));
  bool success = robot_interfaces__msg__HardwareStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_interfaces__msg__HardwareStatus__destroy(robot_interfaces__msg__HardwareStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_interfaces__msg__HardwareStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_interfaces__msg__HardwareStatus__Sequence__init(robot_interfaces__msg__HardwareStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__HardwareStatus * data = NULL;

  if (size) {
    data = (robot_interfaces__msg__HardwareStatus *)allocator.zero_allocate(size, sizeof(robot_interfaces__msg__HardwareStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_interfaces__msg__HardwareStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_interfaces__msg__HardwareStatus__fini(&data[i - 1]);
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
robot_interfaces__msg__HardwareStatus__Sequence__fini(robot_interfaces__msg__HardwareStatus__Sequence * array)
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
      robot_interfaces__msg__HardwareStatus__fini(&array->data[i]);
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

robot_interfaces__msg__HardwareStatus__Sequence *
robot_interfaces__msg__HardwareStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__HardwareStatus__Sequence * array = (robot_interfaces__msg__HardwareStatus__Sequence *)allocator.allocate(sizeof(robot_interfaces__msg__HardwareStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_interfaces__msg__HardwareStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_interfaces__msg__HardwareStatus__Sequence__destroy(robot_interfaces__msg__HardwareStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_interfaces__msg__HardwareStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_interfaces__msg__HardwareStatus__Sequence__are_equal(const robot_interfaces__msg__HardwareStatus__Sequence * lhs, const robot_interfaces__msg__HardwareStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_interfaces__msg__HardwareStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_interfaces__msg__HardwareStatus__Sequence__copy(
  const robot_interfaces__msg__HardwareStatus__Sequence * input,
  robot_interfaces__msg__HardwareStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_interfaces__msg__HardwareStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_interfaces__msg__HardwareStatus * data =
      (robot_interfaces__msg__HardwareStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_interfaces__msg__HardwareStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_interfaces__msg__HardwareStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_interfaces__msg__HardwareStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
