# coding: utf-8

import numpy as np
from geometry_msgs.msg import *

from numpy_ros import to_numpy

# TODO find a more efficient way to test this

def test_vector3_to_numpy():
    message = Vector3(1.0, 2.0, 3.0)
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float64


def test_vector3stamped_to_numpy():
    message = Vector3Stamped(vector=Vector3(1.0, 2.0, 3.0))
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float64


def test_point_to_numpy():
    message = Point(1.0, 2.0, 3.0)
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float64


def test_pointstamped_to_numpy():
    message = PointStamped(point=Point(1.0, 2.0, 3.0))
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float64


def test_point32_to_numpy():
    message = Point32(1.0, 2.0, 3.0)
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float32