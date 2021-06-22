# -*- coding: utf-8 -*-

"""Conversion handlers for the ROS geometry_msgs package."""

# TODO documentation

import collections
import warnings

import numpy as np

# numpy-quaternion warns if numba or scipy are not installed
# TODO what is the best way to deal with this?
with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    import quaternion

from numpy_ros.conversions import (
    converts_to_numpy, converts_to_message, to_message
)

try:
    from geometry_msgs.msg import (
        Accel, AccelStamped, AccelWithCovariance, AccelWithCovarianceStamped,
        Inertia, InertiaStamped, Point, Point32, PointStamped, Polygon,
        PolygonStamped, Pose, PoseArray, PoseStamped, PoseWithCovariance,
        PoseWithCovarianceStamped, Quaternion, QuaternionStamped, Transform,
        TransformStamped, Twist, TwistStamped, TwistWithCovariance,
        TwistWithCovarianceStamped, Vector3, Vector3Stamped, Wrench,
        WrenchStamped,
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
    PolygonStamped: 'polygon',
    PoseStamped: 'pose',
    PoseWithCovariance: 'pose',
    QuaternionStamped: 'quaternion',
    TransformStamped: 'transform',
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


def cast_to_dtype(array, dtype):
    """Raises a TypeError if `array` cannot be casted to `dtype` without
    loss of precision."""

    min_dtype = np.min_scalar_type(array)

    if not np.can_cast(min_dtype, dtype):
        raise TypeError(f'Cannot safely cast array {array} to dtype {dtype}.')

    return array.astype(dtype)


def _assert_has_shape(array, *shapes):
    """Raises a ValueError if `array` cannot be reshaped into any of
    `shapes`."""

    # Assumes that shapes are tuples and sequences of shapes are lists
    if array.shape not in shapes:
        raise ValueError(
            f'Expected array of shape(s): {shapes}, received {array.shape}.'
        )


@converts_to_numpy(Vector3, Vector3Stamped, Point, PointStamped, Point32)
def vector_to_numpy(message, homogeneous=False):

    message = _unstamp(message)

    data = [message.x, message.y, message.z]

    if homogeneous:
        data.append(1.0)

    dtype = np.float32 if isinstance(message, Point32) else np.float64
    array = np.array(data, dtype=dtype)

    return array


@converts_to_message(Vector3, Point, Point32)
def numpy_to_vector(message_type, array):

    dtype = np.float32 if message_type is Point32 else np.float64
    array = cast_to_dtype(array, dtype)

    _assert_has_shape(array, (3,), (4,))

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
        linear_key: to_message(Vector3, linear),
        angular_key: to_message(Vector3, angular)
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
        covariance):

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

    array = cast_to_dtype(array, np.float64)
    _assert_has_shape(array, (6, 6))

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

    _assert_has_shape(inertia_tensor, (3, 3))
    inertia_tensor = cast_to_dtype(inertia_tensor, np.float64)

    mass_center_message = numpy_to_vector(Vector3, mass_center)

    return message_type(
        m=float(mass),
        com=mass_center_message,
        ixx=inertia_tensor[0, 0],
        ixy=inertia_tensor[0, 1],
        ixz=inertia_tensor[0, 2],
        iyy=inertia_tensor[1, 1],
        iyz=inertia_tensor[2, 1],
        izz=inertia_tensor[2, 2]
    )


@converts_to_numpy(Polygon, PolygonStamped)
def polygon_to_numpy(message, homogeneous=False):

    message = _unstamp(message)

    points = np.array(
        [vector_to_numpy(p, homogeneous=homogeneous) for p in message.points],
        dtype=np.float32
    )

    return points.T


@converts_to_message(Polygon)
def numpy_to_polygon(message_type, points):

    # TODO extend _assert_shape to accept np.newaxis
    if points.ndim != 2 or len(points) not in (3, 4):
        raise ValueError(
            (f'Expected matrix of shape (3, *) or (4, *), received '
             f'{points.shape}.')
        )

    points_msg = []

    for point in np.hsplit(points, points.shape[1]):
        point_msg = numpy_to_vector(Point32, point.squeeze())
        points_msg.append(point_msg)

    return message_type(points_msg)


@converts_to_numpy(Quaternion, QuaternionStamped)
def quaternion_to_numpy(message):

    # TODO add to documentation
    # NOTE: In ROS, the 'w' component of a unit quaternion comes last whereas
    # in np.quaternion, 'w' comes first

    message = _unstamp(message)
    return np.quaternion(message.w, message.x, message.y, message.z)


@converts_to_message(Quaternion)
def numpy_to_quaternion(message_type, numpy_obj):

    # TODO add to documentation
    # NOTE: We assume inputs to follow the np.quaternion convention (i.e.
    # 'w' comes first)

    if isinstance(numpy_obj, quaternion.quaternion):
        numpy_obj = quaternion.as_float_array(numpy_obj)

    else:
        numpy_obj = cast_to_dtype(numpy_obj, np.float64)
        _assert_has_shape(numpy_obj, (4,))

    return message_type(
        x=numpy_obj[1],
        y=numpy_obj[2],
        z=numpy_obj[3],
        w=numpy_obj[0],
    )


@converts_to_numpy(Pose, PoseStamped, Transform, TransformStamped)
def frame_to_numpy(message, homogeneous=False):

    message = _unstamp(message)

    is_pose = isinstance(message, Pose)

    position_message = message.position if is_pose else message.translation
    rotation_message = message.orientation if is_pose else message.rotation

    position = vector_to_numpy(position_message, homogeneous=homogeneous)
    rotation = quaternion_to_numpy(rotation_message)

    if homogeneous:
        as_matrix = np.eye(4, dtype=np.float64)

        as_matrix[:, 3] = position
        as_matrix[:3, :3] = quaternion.as_rotation_matrix(rotation)

        return as_matrix

    return position, rotation


@converts_to_message(Pose, Transform)
def numpy_to_frame(message_type, *args):

    is_pose = message_type is Pose

    position_key = 'position' if is_pose else 'translation'
    rotation_key = 'orientation' if is_pose else 'rotation'

    if len(args) == 1:

        matrix = cast_to_dtype(args[0], np.float64)
        _assert_has_shape(matrix, (4, 4))

        if not np.allclose(matrix[3, :], np.array([0.0, 0.0, 0.0, 1.0])):
            raise ValueError(f'{matrix} is not a homogeneous matrix.')

        position = matrix[:3, 3]
        rotation = quaternion.from_rotation_matrix(matrix[:3, :3])

    elif len(args) == 2:
        position, rotation = args

    else:
        raise ValueError(
            (f'Expected either position (np.ndarray of length 3 or 4) and '
             f'rotation (np.quaternion) or homogeneous transform'
             f'(4x4 np.ndarray), received {args}.')
        )

    kwargs = {
        position_key: numpy_to_vector(Vector3, position),
        rotation_key: numpy_to_quaternion(Quaternion, rotation)
    }

    return message_type(**kwargs)


@converts_to_numpy(PoseWithCovariance, PoseWithCovarianceStamped)
def frame_with_covariance_to_numpy(message, homogeneous=False):

    message = _unstamp(message)

    pose = frame_to_numpy(message.pose, homogeneous=homogeneous)
    covariance = np.array(message.covariance, dtype=np.float64).reshape(6, 6)

    return pose, covariance


@converts_to_message(PoseWithCovariance)
def numpy_to_frame_with_covariance(message_type, pose, covariance):

    covariance_message = numpy_to_covariance(covariance)

    pose_message = numpy_to_frame(
        message_type,
        *pose if isinstance(pose, collections.Sequence) else pose
    )

    return message_type(pose=pose_message, covariance=covariance_message)


@converts_to_numpy(PoseArray)
def pose_array_to_numpy(message, homogeneous=False, as_array=False):

    result = [
        frame_to_numpy(pose, homogeneous=homogeneous) for pose in message.poses
    ]

    if as_array:
        result = np.stack(result, axis=0)

    return result


@converts_to_message(PoseArray)
def numpy_to_pose_array(message_type, numpy_obj):

    if isinstance(numpy_obj, np.ndarray):

        # TODO extend _assert_shape to accept np.newaxis
        if numpy_obj.ndim != 3 or numpy_obj.shape[1:] not in (3, 4):
            raise ValueError(
                (f'Expected array of shape (*, 3, 3) or (*, 4, 4), received '
                 f'{numpy_obj.shape}')
            )

        split = np.split(numpy_obj, len(numpy_obj), axis=0)
        numpy_obj = (item.squeeze() for item in split)

    poses = tuple(numpy_to_frame(Pose, item) for item in numpy_obj)

    return message_type(poses=poses)
