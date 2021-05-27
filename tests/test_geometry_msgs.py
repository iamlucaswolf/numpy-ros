# coding: utf-8

import pytest

import numpy as np
from geometry_msgs.msg import *

from numpy_ros import to_numpy, to_message

def test_numpy_to_vector():
    array = np.array([1.0, 2.0, 3.0])
    as_message = to_message(Vector3, array)

    assert as_message == Vector3(1.0, 2.0, 3.0)


def test_numpy_to_vector_hom():
    array = np.array([1.0, 2.0, 3.0, 1.0])
    as_message = to_message(Vector3, array)

    assert as_message == Vector3(1.0, 2.0, 3.0)


def test_numpy_to_vector_hom_invalid():
    array = np.array([1.0, 2.0, 3.0, 0.0])

    with pytest.raises(ValueError):
        to_message(Vector3, array)


def test_numpy_to_vector_invalid_types():
    array = np.array([3.14, 'foo', object()])

    with pytest.raises(TypeError):
        to_message(Vector3, array)

    
def test_numpy_to_vector_invalid_dtype():

    # 2 ** 24 + 1 is representable as a float64, but not float32
    array = np.array([2 ** 24 + 1, 0.0, 0.0], dtype=np.float64)

    with pytest.raises(TypeError):
        to_message(Point32, array)



# TODO find a more efficient way to test this

def test_vector3_to_numpy():
    message = Vector3(1.0, 2.0, 3.0)
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float64


def test_vector3_to_numpy_hom():
    message = Vector3(1.0, 2.0, 3.0)
    as_array = to_numpy(message, homogeneous=True)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0, 1.0]))
    assert as_array.dtype == np.float64


def test_vector3stamped_to_numpy():
    message = Vector3Stamped(vector=Vector3(1.0, 2.0, 3.0))
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float64


def test_vector3stamped_to_numpy_hom():
    message = Vector3Stamped(vector=Vector3(1.0, 2.0, 3.0))
    as_array = to_numpy(message, homogeneous=True)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0, 1.0]))
    assert as_array.dtype == np.float64


def test_point_to_numpy():
    message = Point(1.0, 2.0, 3.0)
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float64


def test_point_to_numpy_hom():
    message = Point(1.0, 2.0, 3.0)
    as_array = to_numpy(message, homogeneous=True)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0, 1.0]))
    assert as_array.dtype == np.float64


def test_pointstamped_to_numpy():
    message = PointStamped(point=Point(1.0, 2.0, 3.0))
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float64


def test_pointstamped_to_numpy_hom():
    message = PointStamped(point=Point(1.0, 2.0, 3.0))
    as_array = to_numpy(message, homogeneous=True)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0, 1.0]))
    assert as_array.dtype == np.float64


def test_point32_to_numpy():
    message = Point32(1.0, 2.0, 3.0)
    as_array = to_numpy(message)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0]))
    assert as_array.dtype == np.float32


def test_point32_to_numpy_hom():
    message = Point32(1.0, 2.0, 3.0)
    as_array = to_numpy(message, homogeneous=True)

    assert np.all(as_array == np.array([1.0, 2.0, 3.0, 1.0]))
    assert as_array.dtype == np.float32