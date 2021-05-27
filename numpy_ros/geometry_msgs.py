# coding: utf-8

"""Conversion handlers for the ROS geometry_msgs package."""

import numpy as np

from numpy_ros.conversions import converts_to_numpy, converts_to_message

try:
    from geometry_msgs.msg import (
        Vector3, Vector3Stamped, Point, Point32, PointStamped
    )

except ImportError:
    raise ImportError(
        'Could not import geometry_msgs. Is the ROS package installed?'
    )


@converts_to_numpy(Vector3, Vector3Stamped, Point, PointStamped, Point32)
def vector_to_numpy(message, homogeneous=False):
    """Converts 3d vector mesage types to a flat array."""

    if isinstance(message, Vector3Stamped):
        message = message.vector

    elif isinstance(message, PointStamped):
        message = message.point

    data = [message.x, message.y, message.z]

    if homogeneous:
        data.append(1.0)

    dtype = np.float32 if isinstance(message, Point32) else np.float64
    array = np.array(data, dtype=dtype)

    return array


@converts_to_message(Vector3, Point, Point32)
def numpy_to_vector(message_type, array):
    """Converts a 3d vector representation to a ROS message"""

    if array.shape not in ((3,), (4,)):
        raise ValueError(
            f'Expected array of shape (3,) or (4,), received {array.shape}.'
        )

    # TODO we will probably need this elsewhere...
    # Check that every element of the array can be converted to the desired
    # dtype, without losing precision

    dtype = np.float32 if message_type is Point32 else np.float64
    min_dtype = np.min_scalar_type(array)

    if not np.can_cast(min_dtype, dtype):
        raise TypeError(f'Cannot safely cast array {array} to dtype {dtype}.')
    
    if len(array) == 4 and not np.isclose(array[3], 1.0):
        raise ValueError(
            (f'Input array has four components, but last component is '
             f'{array[3]:.2} != 1.')
        )

    return message_type(*array[:3])