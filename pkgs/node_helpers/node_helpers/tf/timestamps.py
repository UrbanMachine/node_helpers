from typing import cast

from builtin_interfaces.msg import Time
from rclpy.time import Time as RclpyTime

CONVERSION_CONSTANT = 10**9


def ros_stamp_to_unix_timestamp(stamp: Time | RclpyTime) -> float:
    """Convert a ros2 Time message to a unix timestamp float"""
    rclpy_msg = RclpyTime.from_msg(stamp) if isinstance(stamp, Time) else stamp
    return cast(float, rclpy_msg.nanoseconds / CONVERSION_CONSTANT)


def unix_timestamp_to_ros(stamp: float) -> Time:
    """Convert a unix timestamp to a ros2 Time message"""
    seconds = int(stamp)
    nano_seconds = int((stamp - seconds) * CONVERSION_CONSTANT)
    return Time(sec=seconds, nanosec=nano_seconds)


def is_newer(a: Time, b: Time) -> bool:
    """Check if a > b"""
    return cast(bool, a.sec > b.sec or (a.sec == b.sec and a.nanosec > b.nanosec))


def is_older(a: Time, b: Time) -> bool:
    """Check if a < b"""
    return cast(bool, a.sec < b.sec or (a.sec == b.sec and a.nanosec < b.nanosec))
