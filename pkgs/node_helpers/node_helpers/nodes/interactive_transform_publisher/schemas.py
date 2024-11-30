from dataclasses import dataclass

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from pydantic import BaseModel
from std_msgs.msg import Header

from node_helpers.ros2_numpy import msgify
from node_helpers.tf import ConstantStaticTransformBroadcaster


class TransformModel(BaseModel):
    """A serializable Transform"""

    parent: str
    child: str
    created_via_api: bool = False
    """If True, this transform was created via the ROS API of this node.
    If False, this transform originated from a parameters file.

    This information is used to automatically remove stale transforms from the file
    if they are removed from configuration, but keep them if they were created via api.
    """

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def to_msg(self, stamp: Time) -> TransformStamped:
        return TransformStamped(
            header=Header(frame_id=self.parent, stamp=stamp),
            child_frame_id=self.child,
            transform=Transform(
                translation=msgify(Vector3, np.array(self.translation)),
                rotation=msgify(Quaternion, np.array(self.rotation)),
            ),
        )

    @property
    def marker_name(self) -> str:
        return f"Transform {self.parent} -> {self.child}"


@dataclass
class TransformDescription:
    """A state object to hold the Transform and its respective static broadcaster"""

    model: TransformModel
    broadcaster: ConstantStaticTransformBroadcaster


class TransformsFile(BaseModel):
    """The configuration file format"""

    transforms: list[TransformModel]
