# coding: utf-8

"""Conversion handlers for the ROS geometry_msgs package."""

import numpy as np

from numpy_ros.conversions import converts_to_numpy

try:
    from geometry_msgs.msg import (
        Vector3, Vector3Stamped, Point, Point32, PointStamped
    )

except ImportError:
    raise ImportError(
        'Could not import geometry_msgs. Is the ROS package installed?'
    )


@converts_to_numpy(Vector3, Vector3Stamped, Point, PointStamped, Point32)
def vector_to_numpy(message, as_homogeneous=False):
    """Converts 3d vector mesage types to a flat array."""

    if isinstance(message, Vector3Stamped):
        message = message.vector

    elif isinstance(message, PointStamped):
        message = message.point

    data = [message.x, message.y, message.z]

    if as_homogeneous:
        data.append(1.0)

    dtype = np.float32 if isinstance(message, Point32) else np.float64
    array = np.array(data, dtype=dtype)

    return array