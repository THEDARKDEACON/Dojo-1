# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_interfaces:msg/MotorCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorCommand(type):
    """Metaclass of message 'MotorCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'VELOCITY_MODE': 0,
        'PWM_MODE': 1,
        'POSITION_MODE': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_interfaces.msg.MotorCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_command

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'VELOCITY_MODE': cls.__constants['VELOCITY_MODE'],
            'PWM_MODE': cls.__constants['PWM_MODE'],
            'POSITION_MODE': cls.__constants['POSITION_MODE'],
        }

    @property
    def VELOCITY_MODE(self):
        """Message constant 'VELOCITY_MODE'."""
        return Metaclass_MotorCommand.__constants['VELOCITY_MODE']

    @property
    def PWM_MODE(self):
        """Message constant 'PWM_MODE'."""
        return Metaclass_MotorCommand.__constants['PWM_MODE']

    @property
    def POSITION_MODE(self):
        """Message constant 'POSITION_MODE'."""
        return Metaclass_MotorCommand.__constants['POSITION_MODE']


class MotorCommand(metaclass=Metaclass_MotorCommand):
    """
    Message class 'MotorCommand'.

    Constants:
      VELOCITY_MODE
      PWM_MODE
      POSITION_MODE
    """

    __slots__ = [
        '_header',
        '_control_mode',
        '_left_velocity',
        '_right_velocity',
        '_left_pwm',
        '_right_pwm',
        '_left_position',
        '_right_position',
        '_max_velocity',
        '_max_acceleration',
        '_emergency_stop',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'control_mode': 'uint8',
        'left_velocity': 'double',
        'right_velocity': 'double',
        'left_pwm': 'int16',
        'right_pwm': 'int16',
        'left_position': 'double',
        'right_position': 'double',
        'max_velocity': 'double',
        'max_acceleration': 'double',
        'emergency_stop': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.control_mode = kwargs.get('control_mode', int())
        self.left_velocity = kwargs.get('left_velocity', float())
        self.right_velocity = kwargs.get('right_velocity', float())
        self.left_pwm = kwargs.get('left_pwm', int())
        self.right_pwm = kwargs.get('right_pwm', int())
        self.left_position = kwargs.get('left_position', float())
        self.right_position = kwargs.get('right_position', float())
        self.max_velocity = kwargs.get('max_velocity', float())
        self.max_acceleration = kwargs.get('max_acceleration', float())
        self.emergency_stop = kwargs.get('emergency_stop', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.control_mode != other.control_mode:
            return False
        if self.left_velocity != other.left_velocity:
            return False
        if self.right_velocity != other.right_velocity:
            return False
        if self.left_pwm != other.left_pwm:
            return False
        if self.right_pwm != other.right_pwm:
            return False
        if self.left_position != other.left_position:
            return False
        if self.right_position != other.right_position:
            return False
        if self.max_velocity != other.max_velocity:
            return False
        if self.max_acceleration != other.max_acceleration:
            return False
        if self.emergency_stop != other.emergency_stop:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def control_mode(self):
        """Message field 'control_mode'."""
        return self._control_mode

    @control_mode.setter
    def control_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'control_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'control_mode' field must be an unsigned integer in [0, 255]"
        self._control_mode = value

    @builtins.property
    def left_velocity(self):
        """Message field 'left_velocity'."""
        return self._left_velocity

    @left_velocity.setter
    def left_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_velocity = value

    @builtins.property
    def right_velocity(self):
        """Message field 'right_velocity'."""
        return self._right_velocity

    @right_velocity.setter
    def right_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_velocity = value

    @builtins.property
    def left_pwm(self):
        """Message field 'left_pwm'."""
        return self._left_pwm

    @left_pwm.setter
    def left_pwm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_pwm' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'left_pwm' field must be an integer in [-32768, 32767]"
        self._left_pwm = value

    @builtins.property
    def right_pwm(self):
        """Message field 'right_pwm'."""
        return self._right_pwm

    @right_pwm.setter
    def right_pwm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_pwm' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'right_pwm' field must be an integer in [-32768, 32767]"
        self._right_pwm = value

    @builtins.property
    def left_position(self):
        """Message field 'left_position'."""
        return self._left_position

    @left_position.setter
    def left_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_position = value

    @builtins.property
    def right_position(self):
        """Message field 'right_position'."""
        return self._right_position

    @right_position.setter
    def right_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_position = value

    @builtins.property
    def max_velocity(self):
        """Message field 'max_velocity'."""
        return self._max_velocity

    @max_velocity.setter
    def max_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_velocity = value

    @builtins.property
    def max_acceleration(self):
        """Message field 'max_acceleration'."""
        return self._max_acceleration

    @max_acceleration.setter
    def max_acceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_acceleration' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_acceleration' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_acceleration = value

    @builtins.property
    def emergency_stop(self):
        """Message field 'emergency_stop'."""
        return self._emergency_stop

    @emergency_stop.setter
    def emergency_stop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'emergency_stop' field must be of type 'bool'"
        self._emergency_stop = value
