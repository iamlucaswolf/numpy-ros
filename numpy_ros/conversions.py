# coding: utf-8

from genpy.message import Message
import numpy as np

# Maps ROS message types to conversion functions
_to_numpy = {}
_to_message_handlers = {}

def to_numpy(message: Message, *args, **kwargs):
    """Converts a ROS message into a NumPy representation."""

    convert = _to_numpy.get(message.__class__)

    if not convert:
        raise TypeError(
            (f'Cannot convert unknown message type {message.__class__}. ')
            (f'To register a custom conversion function, see the ')
            (f'`converts_to_numpy` decorator')
        )

    return convert(message, *args, **kwargs)


def converts_to_numpy(*args):
    """Decorator to register a custom Message-to-NumPy conversion function"""

    def decorator(function):

        for message_type in args:
            _to_numpy[message_type] = function
        
        return function

    return decorator