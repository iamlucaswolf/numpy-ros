# coding: utf-8

from numpy_ros.geometry_msgs import vector_to_numpy
import warnings

import pytest
import numpy as np
from geometry_msgs.msg import *

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    import quaternion

from numpy_ros import to_numpy, to_message


@pytest.fixture()
def vector3():
    return Vector3(1.0, 2.0, 3.0)


@pytest.fixture()
def accel(vector3):
    return Accel(vector3, vector3)


@pytest.fixture()
def accel_stamped(accel):
    return AccelStamped(accel=accel)


@pytest.fixture()
def twist(vector3):
    return Twist(vector3, vector3)


@pytest.fixture()
def twist_stamped(twist):
    return TwistStamped(twist=twist)


@pytest.fixture()
def wrench(vector3):
    return Wrench(vector3, vector3)


@pytest.fixture
def wrench_stamped(wrench):
    return WrenchStamped(wrench=wrench)


@pytest.fixture()
def vector_array():
    return np.array([1.0, 2.0, 3.0])


def array_equal(array, other):
    return np.array_equal(array, other) and array.dtype == other.dtype

##
## Accel, AccelStamped
##

def test_accel_to_numpy(accel, vector_array):
    linear, angular = to_numpy(accel)

    assert array_equal(linear, vector_array)
    assert array_equal(angular, vector_array)


def test_accel_stamped_to_numpy(accel_stamped, vector_array):
    test_accel_to_numpy(accel_stamped.accel, vector_array)


def test_numpy_to_accel(vector_array, vector3):
    message = to_message(Accel, vector_array, vector_array)

    assert isinstance(message, Accel)
    
    assert isinstance(message.linear, Vector3)
    assert message.linear == vector3
    
    assert isinstance(message.angular, Vector3)
    assert message.angular == vector3

##
## Twist, TwistStamped
##

def test_twist_to_numpy(twist, vector_array):
    linear, angular = to_numpy(twist)

    assert array_equal(linear, vector_array)
    assert array_equal(angular, vector_array)


def test_twist_stamped_to_numpy(twist_stamped, vector_array):
    test_twist_to_numpy(twist_stamped.twist, vector_array)


def test_numpy_to_twist(vector_array, vector3):
    message = to_message(Twist, vector_array, vector_array)

    assert isinstance(message, Twist)
    
    assert isinstance(message.linear, Vector3)
    assert message.linear == vector3
    
    assert isinstance(message.angular, Vector3)
    assert message.angular == vector3


##
## Wrench, WrenchStamped
##


def test_wrench_to_numpy(wrench, vector_array):
    force, torque = to_numpy(wrench)

    assert array_equal(force, vector_array)
    assert array_equal(torque, vector_array)


def test_wrench_stamped_to_numpy(wrench_stamped, vector_array):
    test_wrench_to_numpy(wrench_stamped.wrench, vector_array)


def test_numpy_to_wrench(vector_array, vector3):
    message = to_message(Wrench, vector_array, vector_array)

    assert isinstance(message, Wrench)
    
    assert isinstance(message.force, Vector3)
    assert message.force == vector3
    
    assert isinstance(message.torque, Vector3)
    assert message.torque == vector3


def test_quaternion_to_numpy():
    message = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    as_array = to_numpy(message)

    assert as_array.dtype == np.quaternion
    assert as_array == np.quaternion(1.0, 0.0, 0.0, 0.0)


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