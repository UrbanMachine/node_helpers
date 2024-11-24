import unittest

import numpy as np
import numpy.typing as npt
import node_helpers.ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2, PointField


class TestPointClouds(unittest.TestCase):
    def make_array(self, npoints: int) -> npt.NDArray[np.float64]:
        points_arr = np.zeros(
            (npoints,),
            dtype=[
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("r", np.uint8),
                ("g", np.uint8),
                ("b", np.uint8),
            ],
        )
        points_arr["x"] = np.random.random((npoints,))
        points_arr["y"] = np.random.random((npoints,))
        points_arr["z"] = np.random.random((npoints,))
        points_arr["r"] = np.floor(np.random.random((npoints,)) * 255)
        points_arr["g"] = 0
        points_arr["b"] = 255

        return points_arr

    def test_convert_dtype(self) -> None:
        fields = [
            PointField(name="x", offset=0, count=1, datatype=PointField.FLOAT32),
            PointField(name="y", offset=4, count=1, datatype=PointField.FLOAT32),
        ]
        dtype = np.dtype([("x", np.float32), ("y", np.float32)])
        conv_fields = rnp.msgify(PointField, dtype, plural=True)
        assert fields == conv_fields, "dtype->Pointfield Failed with simple values"

        conv_dtype = rnp.numpify(fields, point_step=8)
        assert dtype == conv_dtype, "dtype->Pointfield Failed with simple values"

    def test_convert_dtype_inner(self) -> None:
        fields = [
            PointField(name="x", offset=0, count=1, datatype=PointField.FLOAT32),
            PointField(name="y", offset=4, count=1, datatype=PointField.FLOAT32),
            PointField(name="vectors", offset=8, count=3, datatype=PointField.FLOAT32),
        ]

        dtype = np.dtype(
            [("x", np.float32), ("y", np.float32), ("vectors", np.float32, (3,))]
        )

        conv_fields = rnp.msgify(PointField, dtype, plural=True)
        assert fields == conv_fields, "dtype->Pointfield with inner dimensions"

        conv_dtype = rnp.numpify(fields, point_step=8)
        assert dtype == conv_dtype, "Pointfield->dtype with inner dimensions"

    def test_roundtrip(self) -> None:
        points_arr = self.make_array(100)
        cloud_msg = rnp.msgify(PointCloud2, points_arr)
        new_points_arr = rnp.numpify(cloud_msg)

        np.testing.assert_equal(points_arr, new_points_arr)

    def test_roundtrip_numpy(self) -> None:
        points_arr = self.make_array(100)
        cloud_msg = rnp.msgify(PointCloud2, points_arr)
        new_points_arr = rnp.numpify(cloud_msg)

        np.testing.assert_equal(points_arr, new_points_arr)

    def test_roundtrip_zero_points(self) -> None:
        """Test to make sure zero point arrays don't raise memoryview.cast(*) errors"""
        points_arr = self.make_array(0)
        cloud_msg = rnp.msgify(PointCloud2, points_arr)
        new_points_arr = rnp.numpify(cloud_msg)

        np.testing.assert_equal(points_arr, new_points_arr)
