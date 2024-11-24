import unittest

import geometry_msgs
import numpy as np
import node_helpers.ros2_numpy as rnp
import tf_transformations as trans


class TestQuat(unittest.TestCase):
    def test_representation(self) -> None:
        q = trans.quaternion_from_euler(0.0, 0.0, 0.0)
        assert np.allclose(q, np.array([0.0, 0.0, 0.0, 1.0]))

    def test_identity_transform(self) -> None:
        h = rnp.numpify(geometry_msgs.msg.Transform())
        assert np.allclose(h, np.eye(4))
