import math

from builtin_interfaces.msg import Time
from node_helpers.tf import timestamps


def test_timestamp_conversions() -> None:
    """Test the timestamp conversion functions"""
    # A python time.time() timestamp
    python_timestamp = 1702493856.576949412

    ros2_timestamp = timestamps.unix_timestamp_to_ros(python_timestamp)
    assert ros2_timestamp.sec == 1702493856
    assert math.isclose(ros2_timestamp.nanosec, 576949412, abs_tol=100)

    python_timestamp_reconverted = timestamps.ros_stamp_to_unix_timestamp(
        ros2_timestamp
    )
    assert python_timestamp_reconverted == python_timestamp


def test_is_newer() -> None:
    """Test the is_newer function"""
    time_a = Time(sec=1702493856, nanosec=576949412)
    time_b = Time(sec=1702493855, nanosec=576949412)

    assert timestamps.is_newer(time_a, time_b) is True
    assert timestamps.is_newer(time_b, time_a) is False


def test_is_older() -> None:
    """Test the is_older function"""
    time_a = Time(sec=1702493856, nanosec=576949412)
    time_b = Time(sec=1702493855, nanosec=576949412)

    assert timestamps.is_older(time_a, time_b) is False
    assert timestamps.is_older(time_b, time_a) is True
