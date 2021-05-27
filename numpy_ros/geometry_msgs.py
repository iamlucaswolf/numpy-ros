# coding: utf-8

"""Conversion handlers for the ROS geometry_msgs package."""

import numpy as np

from numpy_ros.conversions import converts_to_numpy, converts_to_message, to_numpy

try:
    from geometry_msgs.msg import (
        Accel, AccelStamped, AccelWithCovariance, AccelWithCovarianceStamped,
        Inertia, InertiaStamped, Point, Point32, PointStamped, Twist, 
        TwistStamped, TwistWithCovariance, TwistWithCovarianceStamped, Vector3,
        Vector3Stamped,  Wrench, WrenchStamped
    )

except ImportError:
    raise ImportError(
        'Could not import geometry_msgs. Is the ROS package installed?'
    )

_stamped_type_to_attr = {
    AccelStamped: 'accel',
    AccelWithCovarianceStamped: 'accel',
    InertiaStamped: 'inertia',
    PointStamped: 'point',
    TwistStamped: 'twist',
    TwistWithCovarianceStamped: 'twist',
    Vector3Stamped: 'vector',
    WrenchStamped: 'wrench',
}


def _unstamp(message):
    """Unstamps a given message."""
    attr_name = _stamped_type_to_attr.get(message.__class__)
    
    if attr_name:
        message = getattr(message, attr_name)

    return message


def _assert_is_castable(array, dtype):
    """Checks that every element of the array can be converted to the desired
    dtype, without loss of precision."""

    min_dtype = np.min_scalar_type(array)

    if not np.can_cast(min_dtype, dtype):
        raise TypeError(f'Cannot safely cast array {array} to dtype {dtype}.')


@converts_to_numpy(Vector3, Vector3Stamped, Point, PointStamped, Point32)
def vector_to_numpy(message, homogeneous=False):
    """Converts 3d vector mesage types to a flat array."""

    message = _unstamp(message)

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

    dtype = np.float32 if message_type is Point32 else np.float64
    _assert_is_castable(array, dtype)
    
    if len(array) == 4 and not np.isclose(array[3], 1.0):
        raise ValueError(
            (f'Input array has four components, but last component is '
             f'{array[3]:.2} != 1.')
        )

    return message_type(*array[:3])


@converts_to_numpy(
    Accel, 
    AccelStamped, 
    Twist, 
    TwistStamped, 
    Wrench, 
    WrenchStamped
)
def kinematics_to_numpy(message, homogeneous=False):

    message = _unstamp(message)

    is_wrench = isinstance(message, Wrench)

    linear_message = message.force if is_wrench else message.linear
    angular_message = message.torque if is_wrench else message.angular

    linear = vector_to_numpy(linear_message, homogeneous=homogeneous)
    angular = vector_to_numpy(angular_message, homogeneous=homogeneous)

    return linear, angular


@converts_to_message(Accel, Twist, Wrench)
def numpy_to_kinamatics(message_type, linear, angular):

    is_wrench = message_type is Wrench

    linear_key = 'force' if is_wrench else 'linear'
    angular_key = 'torque' if is_wrench else 'angular'

    kwargs = {
        linear_key: numpy_to_vector(linear),
        angular_key: numpy_to_vector(angular)
    }

    return message_type(**kwargs)


@converts_to_numpy(
    AccelWithCovariance,
    AccelWithCovarianceStamped,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
)
def kinematics_with_covariance_to_numpy(message, homogeneous=False):
    
    message = _unstamp(message)

    is_accel = isinstance(message, AccelWithCovariance)
    kinematics_message = message.accel if is_accel else message.twist

    linear, angular = kinematics_to_numpy(
        kinematics_message, 
        homogeneous=homogeneous
    )

    covariance = np.array(message.covariance, dtype=np.float64).reshape(6, 6)

    return linear, angular, covariance


@converts_to_message(AccelWithCovariance, TwistWithCovariance)
def numpy_to_kinematics_with_covariance(
    message_type, 
    linear, 
    angular, 
    covariance
    ):

    is_accel = message_type is AccelWithCovariance
    
    kinematics_key = 'accel' if is_accel else 'twist'
    kinematics_message_type = Accel if is_accel else Twist

    kinematics_message = numpy_to_kinamatics(
        kinematics_message_type, 
        linear,
        angular
    )

    covariance_message = numpy_to_covariance(covariance)

    kwargs = {
        kinematics_key: kinematics_message,
        'covariance': covariance_message
    }

    return message_type(**kwargs)


def numpy_to_covariance(array):

    _assert_is_castable(array, np.float64)

    if array.shape != (6,6):
        raise ValueError(
            (f'Expected covariance matrix of shape (6,6), received '
             f'{array.shape}.')
        )

    return tuple(array.flatten())


@converts_to_numpy(Inertia, InertiaStamped)
def inertia_to_numpy(message, homogeneous=False):
    
    message = _unstamp(message)

    mass = message.m
    mass_center = vector_to_numpy(message.com, homogeneous=homogeneous)
    inertia_tensor = np.array([
        [message.ixx, message.ixy, message.ixz],
        [message.ixy, message.iyy, message.iyz],
        [message.ixz, message.iyz, message.izz],
    ])

    return mass, mass_center, inertia_tensor


@converts_to_message(Inertia)
def numpy_to_inertia(message_type, mass, mass_center, inertia_tensor):

    _assert_is_castable(mass, np.float64)
    _assert_is_castable(inertia_tensor, np.float64)
    
    mass_center_message = numpy_to_vector(Vector3, mass_center)

    if inertia_tensor.shape != (3, 3):
        raise ValueError(
            (f'Expected inertia tensor of shape (6,6), received '
             f'{inertia_tensor.shape}.')
        )

    return message_type(
        m = float(mass),
        com = mass_center_message,
        ixx = inertia_tensor[0, 0],
        ixy = inertia_tensor[0, 1],
        ixz = inertia_tensor[0, 2],
        iyy = inertia_tensor[1, 1],
        iyz = inertia_tensor[2, 1],
        izz = inertia_tensor[2, 2]
    )