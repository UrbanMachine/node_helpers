import node_helpers.ros2_numpy as rnp
import numpy as np
from rclpy.clock import Clock
from sensor_msgs.msg import LaserScan


def test_to_and_from_laserscan() -> None:
    # Create a dummy LaserScan message
    scan = LaserScan()
    scan.header.stamp = Clock().now().to_msg()
    scan.header.frame_id = "lidar_2d"
    scan.range_min = 0.01
    scan.range_max = 200.0
    scan.angle_increment = np.radians(0.1)
    scan.angle_min = -np.pi
    scan.angle_max = np.pi - scan.angle_increment
    scan.scan_time = 0.0
    scan.time_increment = 0.0
    scan.ranges = np.full(3600, 10.0, dtype="f4").tolist()
    scan.intensities = np.full(3600, 5, dtype="f4").tolist()
    laserscan_array = rnp.numpify(
        scan,
        remove_invalid_ranges=False,
        include_ranges_and_intensities=True,
    )

    assert laserscan_array.shape[0], 3600
    np.testing.assert_array_equal(laserscan_array["ranges"], np.array(scan.ranges))
    np.testing.assert_array_equal(
        laserscan_array["intensities"], np.array(scan.intensities)
    )

    laserscan_array_without_ranges_and_intensities = rnp.numpify(scan)

    laserscan_msg = rnp.msgify(
        LaserScan,
        laserscan_array_without_ranges_and_intensities,
        scan.header,
        scan.scan_time,
        scan.time_increment,
    )

    assert np.isclose(scan.angle_min, laserscan_msg.angle_min)
    assert np.isclose(laserscan_msg.angle_increment, scan.angle_increment)
    assert np.isclose(laserscan_msg.angle_max, scan.angle_max)
    assert np.isclose(laserscan_msg.scan_time, scan.scan_time)
    assert np.isclose(laserscan_msg.time_increment, scan.time_increment)

    assert laserscan_msg.header.frame_id == scan.header.frame_id

    assert len(laserscan_msg.ranges) == 3600
    assert len(laserscan_msg.intensities) == 3600
    np.testing.assert_array_equal(laserscan_array["ranges"], np.array(scan.ranges))
