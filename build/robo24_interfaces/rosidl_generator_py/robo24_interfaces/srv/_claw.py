# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robo24_interfaces:srv/Claw.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Claw_Request(type):
    """Metaclass of message 'Claw_Request'."""

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
            module = import_type_support('robo24_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robo24_interfaces.srv.Claw_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__claw__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__claw__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__claw__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__claw__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__claw__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Claw_Request(metaclass=Metaclass_Claw_Request):
    """Message class 'Claw_Request'."""

    __slots__ = [
        '_pct',
        '_msec',
    ]

    _fields_and_field_types = {
        'pct': 'int16',
        'msec': 'int16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pct = kwargs.get('pct', int())
        self.msec = kwargs.get('msec', int())

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
        if self.pct != other.pct:
            return False
        if self.msec != other.msec:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pct(self):
        """Message field 'pct'."""
        return self._pct

    @pct.setter
    def pct(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pct' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'pct' field must be an integer in [-32768, 32767]"
        self._pct = value

    @builtins.property
    def msec(self):
        """Message field 'msec'."""
        return self._msec

    @msec.setter
    def msec(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'msec' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'msec' field must be an integer in [-32768, 32767]"
        self._msec = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Claw_Response(type):
    """Metaclass of message 'Claw_Response'."""

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
            module = import_type_support('robo24_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robo24_interfaces.srv.Claw_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__claw__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__claw__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__claw__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__claw__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__claw__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Claw_Response(metaclass=Metaclass_Claw_Response):
    """Message class 'Claw_Response'."""

    __slots__ = [
        '_resp',
    ]

    _fields_and_field_types = {
        'resp': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.resp = kwargs.get('resp', str())

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
        if self.resp != other.resp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def resp(self):
        """Message field 'resp'."""
        return self._resp

    @resp.setter
    def resp(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'resp' field must be of type 'str'"
        self._resp = value


class Metaclass_Claw(type):
    """Metaclass of service 'Claw'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robo24_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robo24_interfaces.srv.Claw')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__claw

            from robo24_interfaces.srv import _claw
            if _claw.Metaclass_Claw_Request._TYPE_SUPPORT is None:
                _claw.Metaclass_Claw_Request.__import_type_support__()
            if _claw.Metaclass_Claw_Response._TYPE_SUPPORT is None:
                _claw.Metaclass_Claw_Response.__import_type_support__()


class Claw(metaclass=Metaclass_Claw):
    from robo24_interfaces.srv._claw import Claw_Request as Request
    from robo24_interfaces.srv._claw import Claw_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
