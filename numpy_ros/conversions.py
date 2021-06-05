# -*- coding: utf-8 -*-

"""Registration and dispatching of conversion handlers."""

# TODO documentation

from genpy.message import Message

_to_numpy = {}
_to_message = {}


def to_numpy(message: Message, *args, **kwargs):
    """Converts a ROS message into a NumPy representation."""

    convert = _to_numpy.get(message.__class__)

    if not convert:
        raise TypeError(
            (f'Cannot convert unknown message type {message.__class__}. '
             f'To register a custom conversion handler, see the '
             f'`converts_to_numpy` decorator')
        )

    return convert(message, *args, **kwargs)


def to_message(message_type, *args, **kwargs):
    """Converts a NumPy representation into the specified ROS message type."""

    convert = _to_message.get(message_type)

    if not convert:
        raise ValueError(
            (f'Cannot convert to unknown message type {message_type}. To '
             f'register a custom conversion handler, see the '
             f'`converts_to_message` decorator')
        )

    return convert(message_type, *args, **kwargs)


def converts_to_numpy(*args):
    """Decorator to register a custom Message-to-NumPy handler."""

    def decorator(function):

        for message_type in args:

            if not issubclass(message_type, Message):
                raise TypeError()

            _to_numpy[message_type] = function

        return function

    return decorator


def converts_to_message(*args):
    """Decorator to register a custom NumPy-to-Message handler."""

    def decorator(function):

        for message_type in args:

            if not issubclass(message_type, Message):
                raise TypeError()

            _to_message[message_type] = function

        return function

    return decorator
