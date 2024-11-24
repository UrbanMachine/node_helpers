import unittest

import numpy as np
import pytest
import node_helpers.ros2_numpy as rnp
from sensor_msgs.msg import Image


class TestImages(unittest.TestCase):
    def test_roundtrip_rgb8(self) -> None:
        arr = np.random.randint(0, 256, size=(240, 360, 3)).astype(np.uint8)
        msg = rnp.msgify(Image, arr, encoding="rgb8")
        arr2 = rnp.numpify(msg)

        np.testing.assert_equal(arr, arr2)

    def test_roundtrip_mono(self) -> None:
        arr = np.random.randint(0, 256, size=(240, 360)).astype(np.uint8)
        msg = rnp.msgify(Image, arr, encoding="mono8")
        arr2 = rnp.numpify(msg)

        np.testing.assert_equal(arr, arr2)

    def test_roundtrip_big_endian(self) -> None:
        arr = np.random.randint(0, 256, size=(240, 360)).astype(">u2")
        msg = rnp.msgify(Image, arr, encoding="mono16")
        assert msg.is_bigendian
        arr2 = rnp.numpify(msg)

        np.testing.assert_equal(arr, arr2)

    def test_roundtrip_little_endian(self) -> None:
        arr = np.random.randint(0, 256, size=(240, 360)).astype("<u2")
        msg = rnp.msgify(Image, arr, encoding="mono16")
        assert not msg.is_bigendian
        arr2 = rnp.numpify(msg)

        np.testing.assert_equal(arr, arr2)

    def test_bad_encodings(self) -> None:
        mono_arr = np.random.randint(0, 256, size=(240, 360)).astype(np.uint8)
        mono_arrf = np.random.randint(0, 256, size=(240, 360)).astype(np.float32)
        rgb_arr = np.random.randint(0, 256, size=(240, 360, 3)).astype(np.uint8)
        rgb_arrf = np.random.randint(0, 256, size=(240, 360, 3)).astype(np.float32)

        with pytest.raises(TypeError):
            rnp.msgify(Image, rgb_arr, encoding="mono8")
        with pytest.raises(TypeError):
            rnp.msgify(Image, mono_arrf, encoding="mono8")

        with pytest.raises(TypeError):
            rnp.msgify(Image, rgb_arrf, encoding="rgb8")
        with pytest.raises(TypeError):
            rnp.msgify(Image, mono_arr, encoding="rgb8")
