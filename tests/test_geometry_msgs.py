# coding: utf-8

import warnings

import pytest
import numpy as np
from geometry_msgs.msg import *

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    import quaternion

from numpy_ros import to_numpy, to_message


@pytest.fixture
def vector():
    return np.array([1.0, 2.0, 3.0])


@pytest.fixture
def vector_hom():
    return np.array([1.0, 2.0, 3.0, 1.0])


@pytest.fixture
def covariance():
    return np.arange(36, dtype=np.float64).reshape(6, 6)


@pytest.fixture
def inertia_tensor():
    return np.array([[1.0, 2.0, 3.0], [2.0, 4.0, 5.0], [3.0, 5.0, 6.0]])


@pytest.fixture
def vector3_msg():
    return Vector3(1.0, 2.0, 3.0)


@pytest.fixture
def vector3_stamped_msg(vector3_msg):
    return Vector3Stamped(vector=vector3_msg)


@pytest.fixture
def point_msg():
    return Point(1.0, 2.0, 3.0)


@pytest.fixture
def point32_msg():
    return Point32(1.0, 2.0, 3.0)


@pytest.fixture
def point_stamped_msg(point_msg):
    return PointStamped(point=point_msg)


@pytest.fixture
def accel_msg(vector3_msg):
    return Accel(vector3_msg, vector3_msg)


@pytest.fixture
def accel_stamped_msg(accel_msg):
    return AccelStamped(accel=accel_msg)


@pytest.fixture
def twist_msg(vector3_msg):
    return Twist(vector3_msg, vector3_msg)


@pytest.fixture
def twist_stamped_msg(twist_msg):
    return TwistStamped(twist=twist_msg)


@pytest.fixture
def wrench_msg(vector3_msg):
    return Wrench(vector3_msg, vector3_msg)


@pytest.fixture
def wrench_stamped_msg(wrench_msg):
    return WrenchStamped(wrench=wrench_msg)


@pytest.fixture
def accel_with_covariance_msg(accel_msg):
    return AccelWithCovariance(accel_msg, tuple(float(x) for x in range(36)))


@pytest.fixture
def accel_with_covariance_stamped_msg(accel_with_covariance_msg):
    return AccelWithCovarianceStamped(accel=accel_with_covariance_msg)


@pytest.fixture
def twist_with_covariance_msg(twist_msg):
    return TwistWithCovariance(twist_msg, tuple(float(x) for x in range(36)))


@pytest.fixture
def twist_with_covariance_stamped_msg(twist_with_covariance_msg):
    return TwistWithCovarianceStamped(twist=twist_with_covariance_msg)


@pytest.fixture
def inertia_msg(vector3_msg):
    return Inertia(
        m=0.0,
        com=vector3_msg,
        ixx=1.0,
        ixy=2.0,
        ixz=3.0,
        iyy=4.0,
        iyz=5.0,
        izz=6.0,
    )


@pytest.fixture
def inertia_stamped_msg(inertia_msg):
    return InertiaStamped(inertia_msg)


def array_equal(array, other):
    return np.array_equal(array, other) and array.dtype == other.dtype


# Accel, AccelStamped

def test_accel_to_numpy(accel_msg, vector):
    linear, angular = to_numpy(accel_msg)

    assert array_equal(linear, vector)
    assert array_equal(angular, vector)


def test_accel_stamped_to_numpy(accel_stamped_msg, vector):
    test_accel_to_numpy(accel_stamped_msg.accel, vector)


def test_numpy_to_accel(vector, vector3_msg):
    message = to_message(Accel, vector, vector)

    assert isinstance(message, Accel)

    assert isinstance(message.linear, Vector3)
    assert message.linear == vector3_msg

    assert isinstance(message.angular, Vector3)
    assert message.angular == vector3_msg


# Twist, TwistStamped


def test_twist_to_numpy(twist_msg, vector):
    linear, angular = to_numpy(twist_msg)

    assert array_equal(linear, vector)
    assert array_equal(angular, vector)


def test_twist_stamped_to_numpy(twist_stamped_msg, vector):
    test_twist_to_numpy(twist_stamped_msg.twist, vector)


def test_numpy_to_twist(vector, vector3_msg):
    message = to_message(Twist, vector, vector)

    assert isinstance(message, Twist)

    assert isinstance(message.linear, Vector3)
    assert message.linear == vector3_msg

    assert isinstance(message.angular, Vector3)
    assert message.angular == vector3_msg


# Wrench, WrenchStamped


def test_wrench_to_numpy(wrench_msg, vector):
    force, torque = to_numpy(wrench_msg)

    assert array_equal(force, vector)
    assert array_equal(torque, vector)


def test_wrench_stamped_to_numpy(wrench_stamped_msg, vector):
    test_wrench_to_numpy(wrench_stamped_msg.wrench, vector)


def test_numpy_to_wrench(vector, vector3_msg):
    message = to_message(Wrench, vector, vector)

    assert isinstance(message, Wrench)

    assert isinstance(message.force, Vector3)
    assert message.force == vector3_msg

    assert isinstance(message.torque, Vector3)
    assert message.torque == vector3_msg


# AccelWithCovariance, AccelWithCovarianceStamped


def test_accel_with_covariance_to_numpy(accel_with_covariance_msg, covariance):
    linear, _, covariance_ = to_numpy(accel_with_covariance_msg)

    test_accel_to_numpy(accel_with_covariance_msg.accel, linear)
    assert array_equal(covariance, covariance_)


def test_accel_with_covariance_stamped_to_numpy(
        accel_with_covariance_stamped_msg,
        covariance
        ):

    test_accel_with_covariance_to_numpy(
        accel_with_covariance_stamped_msg.accel,
        covariance
    )


def test_numpy_to_accel_with_covariance(vector, accel_msg, covariance):
    message = to_message(AccelWithCovariance, vector, vector, covariance)

    assert isinstance(message, AccelWithCovariance)

    assert isinstance(message.accel, Accel)
    assert message.accel == accel_msg

    assert array_equal(np.array(message.covariance).reshape(6, 6), covariance)


# TwistWithCovariance, TwistWithCovarianceStamped


def test_twist_with_covariance_to_numpy(twist_with_covariance_msg, covariance):
    linear, _, covariance_ = to_numpy(twist_with_covariance_msg)

    test_twist_to_numpy(twist_with_covariance_msg.twist, linear)
    assert array_equal(covariance, covariance_)


def test_twist_with_covariance_stamped_to_numpy(
        twist_with_covariance_stamped_msg,
        covariance
        ):

    test_twist_with_covariance_to_numpy(
        twist_with_covariance_stamped_msg.twist,
        covariance
    )


def test_numpy_to_twist_with_covariance(vector, twist_msg, covariance):
    message = to_message(TwistWithCovariance, vector, vector, covariance)

    assert isinstance(message, TwistWithCovariance)

    assert isinstance(message.twist, Twist)
    assert message.twist == twist_msg

    assert array_equal(np.array(message.covariance).reshape(6, 6), covariance)


# Inertia, InertiaStamped


def test_inertia_to_numpy(inertia_msg, vector, inertia_tensor):
    mass, mass_center, inertia_tensor_ = to_numpy(inertia_msg)

    assert mass == 0.0 and isinstance(mass, float)
    assert array_equal(mass_center, vector)
    assert array_equal(inertia_tensor_, inertia_tensor)


def test_inertia_to_numpy_hom(inertia_msg, vector_hom, inertia_tensor):
    mass, mass_center, inertia_tensor_ = to_numpy(
        inertia_msg,
        homogeneous=True
    )

    assert mass == 0.0 and isinstance(mass, float)
    assert array_equal(mass_center, vector_hom)
    assert array_equal(inertia_tensor_, inertia_tensor)


# Quaternion


def test_quaternion_to_numpy():
    message = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    as_array = to_numpy(message)

    assert as_array.dtype == np.quaternion
    assert as_array == np.quaternion(1.0, 0.0, 0.0, 0.0)


# Vector3, Vector3Stamped


def test_numpy_to_vector(vector, vector3_msg):
    as_message = to_message(Vector3, vector)

    assert as_message == vector3_msg


def test_numpy_to_vector_hom(vector_hom, vector3_msg):
    as_message = to_message(Vector3, vector_hom)

    assert as_message == vector3_msg


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


def test_vector3_to_numpy(vector3_msg, vector):
    as_array = to_numpy(vector3_msg)

    assert np.all(as_array == np.array(vector))
    assert as_array.dtype == np.float64


def test_vector3_to_numpy_hom(vector3_msg, vector_hom):
    as_array = to_numpy(vector3_msg, homogeneous=True)
    assert array_equal(as_array, vector_hom)


def test_vector3stamped_to_numpy(vector3_stamped_msg, vector):
    as_array = to_numpy(vector3_stamped_msg)
    assert array_equal(as_array, vector)


def test_vector3stamped_to_numpy_hom(vector3_stamped_msg, vector_hom):
    as_array = to_numpy(vector3_stamped_msg, homogeneous=True)
    assert array_equal(as_array, vector_hom)


def test_point_to_numpy(point_msg, vector):
    as_array = to_numpy(point_msg)
    assert array_equal(as_array, vector)


def test_point_to_numpy_hom(point_msg, vector_hom):
    as_array = to_numpy(point_msg, homogeneous=True)
    assert array_equal(as_array, vector_hom)


def test_pointstamped_to_numpy(point_stamped_msg, vector):
    as_array = to_numpy(point_stamped_msg)
    assert array_equal(as_array, vector)


def test_pointstamped_to_numpy_hom(point_stamped_msg, vector_hom):
    as_array = to_numpy(point_stamped_msg, homogeneous=True)
    assert array_equal(as_array, vector_hom)


def test_point32_to_numpy(point32_msg, vector):
    as_array = to_numpy(point32_msg)

    assert np.all(as_array == vector)
    assert as_array.dtype == np.float32


def test_point32_to_numpy_hom(point32_msg, vector_hom):
    as_array = to_numpy(point32_msg, homogeneous=True)

    assert np.all(as_array == vector_hom)
    assert as_array.dtype == np.float32
