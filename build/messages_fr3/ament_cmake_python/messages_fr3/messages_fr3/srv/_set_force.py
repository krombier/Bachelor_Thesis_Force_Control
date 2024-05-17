# generated from rosidl_generator_py/resource/_idl.py.em
# with input from messages_fr3:srv/SetForce.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetForce_Request(type):
    """Metaclass of message 'SetForce_Request'."""

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
            module = import_type_support('messages_fr3')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'messages_fr3.srv.SetForce_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_force__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_force__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_force__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_force__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_force__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetForce_Request(metaclass=Metaclass_SetForce_Request):
    """Message class 'SetForce_Request'."""

    __slots__ = [
        '_x_force',
        '_y_force',
        '_z_force',
        '_x_torque',
        '_y_torque',
        '_z_torque',
    ]

    _fields_and_field_types = {
        'x_force': 'double',
        'y_force': 'double',
        'z_force': 'double',
        'x_torque': 'double',
        'y_torque': 'double',
        'z_torque': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x_force = kwargs.get('x_force', float())
        self.y_force = kwargs.get('y_force', float())
        self.z_force = kwargs.get('z_force', float())
        self.x_torque = kwargs.get('x_torque', float())
        self.y_torque = kwargs.get('y_torque', float())
        self.z_torque = kwargs.get('z_torque', float())

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
        if self.x_force != other.x_force:
            return False
        if self.y_force != other.y_force:
            return False
        if self.z_force != other.z_force:
            return False
        if self.x_torque != other.x_torque:
            return False
        if self.y_torque != other.y_torque:
            return False
        if self.z_torque != other.z_torque:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x_force(self):
        """Message field 'x_force'."""
        return self._x_force

    @x_force.setter
    def x_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_force' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x_force' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x_force = value

    @builtins.property
    def y_force(self):
        """Message field 'y_force'."""
        return self._y_force

    @y_force.setter
    def y_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_force' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y_force' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y_force = value

    @builtins.property
    def z_force(self):
        """Message field 'z_force'."""
        return self._z_force

    @z_force.setter
    def z_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_force' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'z_force' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._z_force = value

    @builtins.property
    def x_torque(self):
        """Message field 'x_torque'."""
        return self._x_torque

    @x_torque.setter
    def x_torque(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_torque' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x_torque' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x_torque = value

    @builtins.property
    def y_torque(self):
        """Message field 'y_torque'."""
        return self._y_torque

    @y_torque.setter
    def y_torque(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_torque' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y_torque' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y_torque = value

    @builtins.property
    def z_torque(self):
        """Message field 'z_torque'."""
        return self._z_torque

    @z_torque.setter
    def z_torque(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_torque' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'z_torque' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._z_torque = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetForce_Response(type):
    """Metaclass of message 'SetForce_Response'."""

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
            module = import_type_support('messages_fr3')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'messages_fr3.srv.SetForce_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_force__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_force__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_force__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_force__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_force__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetForce_Response(metaclass=Metaclass_SetForce_Response):
    """Message class 'SetForce_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_SetForce(type):
    """Metaclass of service 'SetForce'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('messages_fr3')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'messages_fr3.srv.SetForce')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_force

            from messages_fr3.srv import _set_force
            if _set_force.Metaclass_SetForce_Request._TYPE_SUPPORT is None:
                _set_force.Metaclass_SetForce_Request.__import_type_support__()
            if _set_force.Metaclass_SetForce_Response._TYPE_SUPPORT is None:
                _set_force.Metaclass_SetForce_Response.__import_type_support__()


class SetForce(metaclass=Metaclass_SetForce):
    from messages_fr3.srv._set_force import SetForce_Request as Request
    from messages_fr3.srv._set_force import SetForce_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
