# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_interfaces:msg/RobotState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotState(type):
    """Metaclass of message 'RobotState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'UNKNOWN': 0,
        'INITIALIZING': 1,
        'IDLE': 2,
        'MANUAL_CONTROL': 3,
        'AUTONOMOUS': 4,
        'ERROR': 5,
        'EMERGENCY_STOP': 6,
        'SHUTDOWN': 7,
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
                'robot_interfaces.msg.RobotState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_state

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'UNKNOWN': cls.__constants['UNKNOWN'],
            'INITIALIZING': cls.__constants['INITIALIZING'],
            'IDLE': cls.__constants['IDLE'],
            'MANUAL_CONTROL': cls.__constants['MANUAL_CONTROL'],
            'AUTONOMOUS': cls.__constants['AUTONOMOUS'],
            'ERROR': cls.__constants['ERROR'],
            'EMERGENCY_STOP': cls.__constants['EMERGENCY_STOP'],
            'SHUTDOWN': cls.__constants['SHUTDOWN'],
        }

    @property
    def UNKNOWN(self):
        """Message constant 'UNKNOWN'."""
        return Metaclass_RobotState.__constants['UNKNOWN']

    @property
    def INITIALIZING(self):
        """Message constant 'INITIALIZING'."""
        return Metaclass_RobotState.__constants['INITIALIZING']

    @property
    def IDLE(self):
        """Message constant 'IDLE'."""
        return Metaclass_RobotState.__constants['IDLE']

    @property
    def MANUAL_CONTROL(self):
        """Message constant 'MANUAL_CONTROL'."""
        return Metaclass_RobotState.__constants['MANUAL_CONTROL']

    @property
    def AUTONOMOUS(self):
        """Message constant 'AUTONOMOUS'."""
        return Metaclass_RobotState.__constants['AUTONOMOUS']

    @property
    def ERROR(self):
        """Message constant 'ERROR'."""
        return Metaclass_RobotState.__constants['ERROR']

    @property
    def EMERGENCY_STOP(self):
        """Message constant 'EMERGENCY_STOP'."""
        return Metaclass_RobotState.__constants['EMERGENCY_STOP']

    @property
    def SHUTDOWN(self):
        """Message constant 'SHUTDOWN'."""
        return Metaclass_RobotState.__constants['SHUTDOWN']


class RobotState(metaclass=Metaclass_RobotState):
    """
    Message class 'RobotState'.

    Constants:
      UNKNOWN
      INITIALIZING
      IDLE
      MANUAL_CONTROL
      AUTONOMOUS
      ERROR
      EMERGENCY_STOP
      SHUTDOWN
    """

    __slots__ = [
        '_header',
        '_current_state',
        '_previous_state',
        '_state_change_time',
        '_hardware_ok',
        '_software_ok',
        '_communication_ok',
        '_sensors_ok',
        '_can_move',
        '_can_navigate',
        '_can_perceive',
        '_active_errors',
        '_warnings',
        '_cpu_usage',
        '_memory_usage',
        '_network_usage',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'current_state': 'uint8',
        'previous_state': 'uint8',
        'state_change_time': 'builtin_interfaces/Time',
        'hardware_ok': 'boolean',
        'software_ok': 'boolean',
        'communication_ok': 'boolean',
        'sensors_ok': 'boolean',
        'can_move': 'boolean',
        'can_navigate': 'boolean',
        'can_perceive': 'boolean',
        'active_errors': 'sequence<string>',
        'warnings': 'sequence<string>',
        'cpu_usage': 'double',
        'memory_usage': 'double',
        'network_usage': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.current_state = kwargs.get('current_state', int())
        self.previous_state = kwargs.get('previous_state', int())
        from builtin_interfaces.msg import Time
        self.state_change_time = kwargs.get('state_change_time', Time())
        self.hardware_ok = kwargs.get('hardware_ok', bool())
        self.software_ok = kwargs.get('software_ok', bool())
        self.communication_ok = kwargs.get('communication_ok', bool())
        self.sensors_ok = kwargs.get('sensors_ok', bool())
        self.can_move = kwargs.get('can_move', bool())
        self.can_navigate = kwargs.get('can_navigate', bool())
        self.can_perceive = kwargs.get('can_perceive', bool())
        self.active_errors = kwargs.get('active_errors', [])
        self.warnings = kwargs.get('warnings', [])
        self.cpu_usage = kwargs.get('cpu_usage', float())
        self.memory_usage = kwargs.get('memory_usage', float())
        self.network_usage = kwargs.get('network_usage', float())

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
        if self.current_state != other.current_state:
            return False
        if self.previous_state != other.previous_state:
            return False
        if self.state_change_time != other.state_change_time:
            return False
        if self.hardware_ok != other.hardware_ok:
            return False
        if self.software_ok != other.software_ok:
            return False
        if self.communication_ok != other.communication_ok:
            return False
        if self.sensors_ok != other.sensors_ok:
            return False
        if self.can_move != other.can_move:
            return False
        if self.can_navigate != other.can_navigate:
            return False
        if self.can_perceive != other.can_perceive:
            return False
        if self.active_errors != other.active_errors:
            return False
        if self.warnings != other.warnings:
            return False
        if self.cpu_usage != other.cpu_usage:
            return False
        if self.memory_usage != other.memory_usage:
            return False
        if self.network_usage != other.network_usage:
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
    def current_state(self):
        """Message field 'current_state'."""
        return self._current_state

    @current_state.setter
    def current_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'current_state' field must be an unsigned integer in [0, 255]"
        self._current_state = value

    @builtins.property
    def previous_state(self):
        """Message field 'previous_state'."""
        return self._previous_state

    @previous_state.setter
    def previous_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'previous_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'previous_state' field must be an unsigned integer in [0, 255]"
        self._previous_state = value

    @builtins.property
    def state_change_time(self):
        """Message field 'state_change_time'."""
        return self._state_change_time

    @state_change_time.setter
    def state_change_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'state_change_time' field must be a sub message of type 'Time'"
        self._state_change_time = value

    @builtins.property
    def hardware_ok(self):
        """Message field 'hardware_ok'."""
        return self._hardware_ok

    @hardware_ok.setter
    def hardware_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'hardware_ok' field must be of type 'bool'"
        self._hardware_ok = value

    @builtins.property
    def software_ok(self):
        """Message field 'software_ok'."""
        return self._software_ok

    @software_ok.setter
    def software_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'software_ok' field must be of type 'bool'"
        self._software_ok = value

    @builtins.property
    def communication_ok(self):
        """Message field 'communication_ok'."""
        return self._communication_ok

    @communication_ok.setter
    def communication_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'communication_ok' field must be of type 'bool'"
        self._communication_ok = value

    @builtins.property
    def sensors_ok(self):
        """Message field 'sensors_ok'."""
        return self._sensors_ok

    @sensors_ok.setter
    def sensors_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sensors_ok' field must be of type 'bool'"
        self._sensors_ok = value

    @builtins.property
    def can_move(self):
        """Message field 'can_move'."""
        return self._can_move

    @can_move.setter
    def can_move(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'can_move' field must be of type 'bool'"
        self._can_move = value

    @builtins.property
    def can_navigate(self):
        """Message field 'can_navigate'."""
        return self._can_navigate

    @can_navigate.setter
    def can_navigate(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'can_navigate' field must be of type 'bool'"
        self._can_navigate = value

    @builtins.property
    def can_perceive(self):
        """Message field 'can_perceive'."""
        return self._can_perceive

    @can_perceive.setter
    def can_perceive(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'can_perceive' field must be of type 'bool'"
        self._can_perceive = value

    @builtins.property
    def active_errors(self):
        """Message field 'active_errors'."""
        return self._active_errors

    @active_errors.setter
    def active_errors(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'active_errors' field must be a set or sequence and each value of type 'str'"
        self._active_errors = value

    @builtins.property
    def warnings(self):
        """Message field 'warnings'."""
        return self._warnings

    @warnings.setter
    def warnings(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'warnings' field must be a set or sequence and each value of type 'str'"
        self._warnings = value

    @builtins.property
    def cpu_usage(self):
        """Message field 'cpu_usage'."""
        return self._cpu_usage

    @cpu_usage.setter
    def cpu_usage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cpu_usage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'cpu_usage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._cpu_usage = value

    @builtins.property
    def memory_usage(self):
        """Message field 'memory_usage'."""
        return self._memory_usage

    @memory_usage.setter
    def memory_usage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'memory_usage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'memory_usage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._memory_usage = value

    @builtins.property
    def network_usage(self):
        """Message field 'network_usage'."""
        return self._network_usage

    @network_usage.setter
    def network_usage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'network_usage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'network_usage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._network_usage = value
