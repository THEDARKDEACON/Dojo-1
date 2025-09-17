# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_interfaces:msg/SensorData.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'temperatures'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SensorData(type):
    """Metaclass of message 'SensorData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
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
                'robot_interfaces.msg.SensorData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sensor_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sensor_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sensor_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sensor_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sensor_data

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SensorData(metaclass=Metaclass_SensorData):
    """Message class 'SensorData'."""

    __slots__ = [
        '_header',
        '_left_encoder',
        '_right_encoder',
        '_left_velocity',
        '_right_velocity',
        '_ultrasonic_distance',
        '_ultrasonic_valid',
        '_imu_available',
        '_linear_acceleration',
        '_angular_velocity',
        '_orientation',
        '_battery_voltage',
        '_current_draw',
        '_temperatures',
        '_temperature_labels',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'left_encoder': 'int32',
        'right_encoder': 'int32',
        'left_velocity': 'double',
        'right_velocity': 'double',
        'ultrasonic_distance': 'double',
        'ultrasonic_valid': 'boolean',
        'imu_available': 'boolean',
        'linear_acceleration': 'geometry_msgs/Vector3',
        'angular_velocity': 'geometry_msgs/Vector3',
        'orientation': 'geometry_msgs/Quaternion',
        'battery_voltage': 'double',
        'current_draw': 'double',
        'temperatures': 'sequence<double>',
        'temperature_labels': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.left_encoder = kwargs.get('left_encoder', int())
        self.right_encoder = kwargs.get('right_encoder', int())
        self.left_velocity = kwargs.get('left_velocity', float())
        self.right_velocity = kwargs.get('right_velocity', float())
        self.ultrasonic_distance = kwargs.get('ultrasonic_distance', float())
        self.ultrasonic_valid = kwargs.get('ultrasonic_valid', bool())
        self.imu_available = kwargs.get('imu_available', bool())
        from geometry_msgs.msg import Vector3
        self.linear_acceleration = kwargs.get('linear_acceleration', Vector3())
        from geometry_msgs.msg import Vector3
        self.angular_velocity = kwargs.get('angular_velocity', Vector3())
        from geometry_msgs.msg import Quaternion
        self.orientation = kwargs.get('orientation', Quaternion())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.current_draw = kwargs.get('current_draw', float())
        self.temperatures = array.array('d', kwargs.get('temperatures', []))
        self.temperature_labels = kwargs.get('temperature_labels', [])

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
        if self.left_encoder != other.left_encoder:
            return False
        if self.right_encoder != other.right_encoder:
            return False
        if self.left_velocity != other.left_velocity:
            return False
        if self.right_velocity != other.right_velocity:
            return False
        if self.ultrasonic_distance != other.ultrasonic_distance:
            return False
        if self.ultrasonic_valid != other.ultrasonic_valid:
            return False
        if self.imu_available != other.imu_available:
            return False
        if self.linear_acceleration != other.linear_acceleration:
            return False
        if self.angular_velocity != other.angular_velocity:
            return False
        if self.orientation != other.orientation:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.current_draw != other.current_draw:
            return False
        if self.temperatures != other.temperatures:
            return False
        if self.temperature_labels != other.temperature_labels:
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
    def left_encoder(self):
        """Message field 'left_encoder'."""
        return self._left_encoder

    @left_encoder.setter
    def left_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._left_encoder = value

    @builtins.property
    def right_encoder(self):
        """Message field 'right_encoder'."""
        return self._right_encoder

    @right_encoder.setter
    def right_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._right_encoder = value

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
    def ultrasonic_distance(self):
        """Message field 'ultrasonic_distance'."""
        return self._ultrasonic_distance

    @ultrasonic_distance.setter
    def ultrasonic_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ultrasonic_distance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ultrasonic_distance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ultrasonic_distance = value

    @builtins.property
    def ultrasonic_valid(self):
        """Message field 'ultrasonic_valid'."""
        return self._ultrasonic_valid

    @ultrasonic_valid.setter
    def ultrasonic_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ultrasonic_valid' field must be of type 'bool'"
        self._ultrasonic_valid = value

    @builtins.property
    def imu_available(self):
        """Message field 'imu_available'."""
        return self._imu_available

    @imu_available.setter
    def imu_available(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'imu_available' field must be of type 'bool'"
        self._imu_available = value

    @builtins.property
    def linear_acceleration(self):
        """Message field 'linear_acceleration'."""
        return self._linear_acceleration

    @linear_acceleration.setter
    def linear_acceleration(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'linear_acceleration' field must be a sub message of type 'Vector3'"
        self._linear_acceleration = value

    @builtins.property
    def angular_velocity(self):
        """Message field 'angular_velocity'."""
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'angular_velocity' field must be a sub message of type 'Vector3'"
        self._angular_velocity = value

    @builtins.property
    def orientation(self):
        """Message field 'orientation'."""
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if __debug__:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'orientation' field must be a sub message of type 'Quaternion'"
        self._orientation = value

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'battery_voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._battery_voltage = value

    @builtins.property
    def current_draw(self):
        """Message field 'current_draw'."""
        return self._current_draw

    @current_draw.setter
    def current_draw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_draw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current_draw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current_draw = value

    @builtins.property
    def temperatures(self):
        """Message field 'temperatures'."""
        return self._temperatures

    @temperatures.setter
    def temperatures(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'temperatures' array.array() must have the type code of 'd'"
            self._temperatures = value
            return
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'temperatures' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._temperatures = array.array('d', value)

    @builtins.property
    def temperature_labels(self):
        """Message field 'temperature_labels'."""
        return self._temperature_labels

    @temperature_labels.setter
    def temperature_labels(self, value):
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
                "The 'temperature_labels' field must be a set or sequence and each value of type 'str'"
        self._temperature_labels = value
