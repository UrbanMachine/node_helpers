import node_helpers.ros2_numpy as rnp
import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.serialization import serialize_message


def test_masking() -> None:
    data = -np.ones((30, 30), np.int8)
    data[10:20, 10:20] = 100

    msg = rnp.msgify(OccupancyGrid, data)

    data_out = rnp.numpify(msg)

    assert data_out[5, 5] is np.ma.masked
    np.testing.assert_equal(data_out[10:20, 10:20], 100)


def test_serialization() -> None:
    msg = OccupancyGrid(
        info=MapMetaData(width=3, height=3), data=[0, 0, 0, 0, -1, 0, 0, 0, 0]
    )

    data = rnp.numpify(msg)
    assert data[1, 1] is np.ma.masked
    msg2 = rnp.msgify(OccupancyGrid, data)

    assert msg.info == msg2.info

    msg_ser = serialize_message(msg)
    msg2_ser = serialize_message(msg2)

    assert msg_ser == msg2_ser, "Message serialization survives round-trip"
