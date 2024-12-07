import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time

from node_helpers.ros2_numpy import numpify
from node_helpers.testing.urdf_module_fixture import TFClient, URDFModuleFixture

_TEST_TF_TIMEOUT = Duration(seconds=10)


def validate_coincident_transforms(
    urdf_module: URDFModuleFixture,
    tf_client: TFClient,
    joint_origin: str,
    joint_current: str,
    same_position: bool,
    same_orientation: bool,
) -> None:
    """Test that joints that are expected to be coincident while at 0, are so.
    This means both position and orientation are equivalent.

    :param urdf_module: The URDFModuleFixture instance to use for the test.
    :param tf_client: The TFClient instance to use for the test.
    :param joint_origin: The joint we transform from.
    :param joint_current: The joint we transform to.
    :param same_position: Whether the position of the frames should be the same.
    :param same_orientation: Whether the orientation of the frames should be the same
    """
    assert joint_origin != joint_current
    assert same_position or same_orientation
    transform = tf_client.buffer.lookup_transform(
        joint_origin, joint_current, Time(), timeout=_TEST_TF_TIMEOUT
    )
    position = np.round(numpify(transform.transform.translation), decimals=5)
    rotation = np.round(numpify(transform.transform.rotation), decimals=5)

    if same_position:
        error = (
            f"The frame '{joint_origin}' is expected to be coincident with "
            f"'{joint_current}' when homed! Current offset: {position}, in URDF "
            f"'{urdf_module.parameters.urdf_constant_name}'"
        )
        assert np.isclose(position, (0, 0, 0)).all(), error

    if same_orientation:
        error = (
            f"The frame '{joint_origin}' is expected to be coincident with "
            f"'{joint_current}' when homed! Current rotation quaternion: {rotation}, "
            f"in URDF '{urdf_module.parameters.urdf_constant_name}'"
        )
        assert np.isclose(rotation, (0, 0, 0, 1)).all(), error


def validate_expected_rotation(
    urdf_module: URDFModuleFixture,
    tf_client: TFClient,
    from_frame: str,
    to_frame: str,
    start_point: tuple[float, float, float],
    expected_end_point: tuple[float, float, float],
) -> None:
    """Test that two frames have a specific rotation between them. To do this, the test
    places a point at 'start_point' and then applies the given rotation between the
    frames, and check if it ends up at the 'expected_end_point'.

    :param urdf_module: The URDFModuleFixture instance to use for the test.
    :param tf_client: The TFClient instance to use for the test.
    :param from_frame: The frame we transform from.
    :param to_frame: The frame we transform to.
    :param start_point: The point being transformed.
    :param expected_end_point: The point we expect after transformation.
    :raises ImportError: If the 'scipy' package is not installed.
    """
    try:
        from scipy.spatial.transform import Rotation
    except ImportError as e:
        raise ImportError(
            "The 'scipy' package is required for this test. Please install it using "
            "the following command: 'pip install scipy'."
            "In the future we might replace this dependency with ."
        ) from e

    transform = tf_client.buffer.lookup_transform(
        to_frame, from_frame, Time(), timeout=_TEST_TF_TIMEOUT
    )
    rotation = Rotation.from_quat(numpify(transform.transform.rotation))

    point_after_transformation = np.round(rotation.apply(np.array(start_point)), 4)

    error = (
        f"After applying the rotation between '{from_frame}' and '{to_frame}', the "
        f"point {start_point} was expected to end up at {expected_end_point}, but "
        f"instead ended up at {point_after_transformation}."
    )
    assert np.isclose(point_after_transformation, expected_end_point).all(), error
