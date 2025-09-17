// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_interfaces:msg/MotorCommand.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "robot_interfaces/msg/detail/motor_command__struct.h"
#include "robot_interfaces/msg/detail/motor_command__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool robot_interfaces__msg__motor_command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robot_interfaces.msg._motor_command.MotorCommand", full_classname_dest, 48) == 0);
  }
  robot_interfaces__msg__MotorCommand * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // control_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "control_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->control_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // left_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_pwm
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_pwm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->left_pwm = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // right_pwm
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_pwm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->right_pwm = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // left_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_acceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_acceleration = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // emergency_stop
    PyObject * field = PyObject_GetAttrString(_pymsg, "emergency_stop");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->emergency_stop = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_interfaces__msg__motor_command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MotorCommand */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_interfaces.msg._motor_command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MotorCommand");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_interfaces__msg__MotorCommand * ros_message = (robot_interfaces__msg__MotorCommand *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // control_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->control_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "control_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_pwm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->left_pwm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_pwm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_pwm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->right_pwm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_pwm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_acceleration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_acceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // emergency_stop
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->emergency_stop ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "emergency_stop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
