import geometry_msgs
import node_helpers.ros2_numpy as rnp
import numpy as np
import tf_transformations as trans


def test_representation() -> None:
    q = trans.quaternion_from_euler(0.0, 0.0, 0.0)
    assert np.allclose(q, np.array([0.0, 0.0, 0.0, 1.0]))


def test_identity_transform() -> None:
    h = rnp.numpify(geometry_msgs.msg.Transform())
    assert np.allclose(h, np.eye(4))
