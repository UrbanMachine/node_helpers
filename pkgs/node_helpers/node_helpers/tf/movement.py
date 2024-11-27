from time import time

import numpy as np
import numpy.typing as npt
from builtin_interfaces.msg import Time
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer

from node_helpers.ros2_numpy import numpify
from node_helpers.tf import timestamps
from node_helpers.timing import Timeout


def tf_velocity(
    frame_id: str,
    tf_buffer: Buffer,
    at_time: Time,
    sample_duration: float = 0.005,
    relative_to_frame: str = "base_link",
    tf_timeout: int | None = 10,
) -> npt.NDArray[np.float64]:
    """Returns the velocity in meters per second along the X, Y Z axis of the given
    frame_id.

    :param frame_id: The frame_id to get the velocity of
    :param tf_buffer: The tf buffer to use
    :param at_time: The time to get the velocity at
    :param sample_duration: The duration to sample the velocity over
    :param relative_to_frame: The frame to get the velocity relative to
    :param tf_timeout: The timeout to wait for the transform
    :return: The (X, Y, Z) velicities in meters per second
    :raises ValueError: If the at_time is not a valid timestamp
    """
    if at_time == Time():
        raise ValueError("This function must have a valid timestamp!")

    tf_timeout = tf_timeout or None

    past_time = RclpyTime.from_msg(at_time) - Duration(seconds=sample_duration)
    vector = tf_buffer.lookup_transform_full(
        target_frame=frame_id,
        target_time=past_time,
        source_frame=frame_id,
        source_time=at_time,
        fixed_frame=relative_to_frame,
        timeout=Duration(seconds=tf_timeout),
    ).transform.translation

    # Convert to velocity in meters per second along the wood axis (x)
    return numpify(vector) / sample_duration


def block_until_tfs_are_static(
    frame_ids: list[str],
    tf_buffer: Buffer,
    start_time: float | None = None,
    timeout: int | None = 10,
    time_increment: float = 0.01,
    velocity_tolerance: float = 0.0001,
) -> None:
    """Wait until each of the given TFs are stable and no longer moving

    :param frame_ids: The frame_ids to wait for 0 velocity
    :param tf_buffer: The tf buffer to use
    :param start_time: The minimum time to start waiting for static tfs. If unset,
        the current time is used.
    :param timeout: The maximum time to wait for the tfs to become static.
    :param time_increment: The time increment to use when checking for static tfs.
    :param velocity_tolerance: The "static" tolerance to use when checking for tfs.
    """
    velocity_at_time = start_time or time()
    msg = f"Timed out while waiting for frames to be static: {frame_ids}"
    retry_timeout = (
        Timeout(timeout, raise_error=True, timeout_message=msg) if timeout else True
    )
    while retry_timeout:
        velocities = []
        for frame_id in frame_ids:
            velocity = tf_velocity(
                frame_id=frame_id,
                tf_buffer=tf_buffer,
                at_time=timestamps.unix_timestamp_to_ros(velocity_at_time),
                sample_duration=0.005,
                tf_timeout=timeout,
            )
            velocities.append(round(np.linalg.norm(velocity), 6))

        if all(
            np.linalg.norm(velocity) <= velocity_tolerance for velocity in velocities
        ):
            break

        # Since velocities weren't stable at velocity_at_time, try again but with a time
        # increment
        velocity_at_time += time_increment
