# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_interfaces:msg/HardwareStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HardwareStatus(type):
    """Metaclass of message 'HardwareStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'UNKNOWN': 0,
        'INITIALIZING': 1,
        'READY': 2,
        'ERROR': 3,
        'DISCONNECTED': 4,
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
                'robot_interfaces.msg.HardwareStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hardware_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hardware_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hardware_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hardware_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hardware_status

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
            'READY': cls.__constants['READY'],
            'ERROR': cls.__constants['ERROR'],
            'DISCONNECTED': cls.__constants['DISCONNECTED'],
        }

    @property
    def UNKNOWN(self):
        """Message constant 'UNKNOWN'."""
        return Metaclass_HardwareStatus.__constants['UNKNOWN']

    @property
    def INITIALIZING(self):
        """Message constant 'INITIALIZING'."""
        return Metaclass_HardwareStatus.__constants['INITIALIZING']

    @property
    def READY(self):
        """Message constant 'READY'."""
        return Metaclass_HardwareStatus.__constants['READY']

    @property
    def ERROR(self):
        """Message constant 'ERROR'."""
        return Metaclass_HardwareStatus.__constants['ERROR']

    @property
    def DISCONNECTED(self):
        """Message constant 'DISCONNECTED'."""
        return Metaclass_HardwareStatus.__constants['DISCONNECTED']


class HardwareStatus(metaclass=Metaclass_HardwareStatus):
    """
    Message class 'HardwareStatus'.

    Constants:
      UNKNOWN
      INITIALIZING
      READY
      ERROR
      DISCONNECTED
    """

    __slots__ = [
        '_header',
        '_component_name',
        '_hardware_id',
        '_status',
        '_status_message',
        '_uptime',
        '_last_update',
        '_data_keys',
        '_data_values',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'component_name': 'string',
        'hardware_id': 'string',
        'status': 'uint8',
        'status_message': 'string',
        'uptime': 'double',
        'last_update': 'double',
        'data_keys': 'sequence<string>',
        'data_values': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.component_name = kwargs.get('component_name', str())
        self.hardware_id = kwargs.get('hardware_id', str())
        self.status = kwargs.get('status', int())
        self.status_message = kwargs.get('status_message', str())
        self.uptime = kwargs.get('uptime', float())
        self.last_update = kwargs.get('last_update', float())
        self.data_keys = kwargs.get('data_keys', [])
        self.data_values = kwargs.get('data_values', [])

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
        if self.component_name != other.component_name:
            return False
        if self.hardware_id != other.hardware_id:
            return False
        if self.status != other.status:
            return False
        if self.status_message != other.status_message:
            return False
        if self.uptime != other.uptime:
            return False
        if self.last_update != other.last_update:
            return False
        if self.data_keys != other.data_keys:
            return False
        if self.data_values != other.data_values:
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
    def component_name(self):
        """Message field 'component_name'."""
        return self._component_name

    @component_name.setter
    def component_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'component_name' field must be of type 'str'"
        self._component_name = value

    @builtins.property
    def hardware_id(self):
        """Message field 'hardware_id'."""
        return self._hardware_id

    @hardware_id.setter
    def hardware_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'hardware_id' field must be of type 'str'"
        self._hardware_id = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value

    @builtins.property
    def status_message(self):
        """Message field 'status_message'."""
        return self._status_message

    @status_message.setter
    def status_message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status_message' field must be of type 'str'"
        self._status_message = value

    @builtins.property
    def uptime(self):
        """Message field 'uptime'."""
        return self._uptime

    @uptime.setter
    def uptime(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'uptime' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'uptime' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._uptime = value

    @builtins.property
    def last_update(self):
        """Message field 'last_update'."""
        return self._last_update

    @last_update.setter
    def last_update(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'last_update' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'last_update' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._last_update = value

    @builtins.property
    def data_keys(self):
        """Message field 'data_keys'."""
        return self._data_keys

    @data_keys.setter
    def data_keys(self, value):
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
                "The 'data_keys' field must be a set or sequence and each value of type 'str'"
        self._data_keys = value

    @builtins.property
    def data_values(self):
        """Message field 'data_values'."""
        return self._data_values

    @data_values.setter
    def data_values(self, value):
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
                "The 'data_values' field must be a set or sequence and each value of type 'str'"
        self._data_values = value
