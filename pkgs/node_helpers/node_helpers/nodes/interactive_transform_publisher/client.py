import logging

import numpy as np
import numpy.typing as npt
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Transform, TransformStamped
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Header
from tf2_ros import Buffer

from node_helpers.nodes import HelpfulNode
from node_helpers.qos import qos_profile_transient_reliable_keep_all
from node_helpers.ros2_numpy import msgify, numpify


class InteractiveTransformClient:
    """A helper object for calling APIs on the InteractiveTransformPublisher"""

    def __init__(
        self,
        node: HelpfulNode,
        namespace: str | None = None,
        callback_group: CallbackGroup | None = None,
    ):
        callback_group = callback_group or MutuallyExclusiveCallbackGroup()
        self.namespace = namespace or node.get_namespace().replace("/", "")

        self.update_transform = node.create_publisher(
            TransformStamped,
            f"/{self.namespace}/tf_static_updates",
            qos_profile=qos_profile_transient_reliable_keep_all,
            callback_group=callback_group,
        )
        self.create_transform = node.create_publisher(
            TransformStamped,
            f"/{self.namespace}/tf_static_create",
            qos_profile=qos_profile_transient_reliable_keep_all,
            callback_group=callback_group,
        )

    def adjust_transform(self, tf_buffer: Buffer, adjustment: TransformStamped) -> None:
        """Apply an adjustment to the transform. This is useful for calibration."""
        self.adjust_transform_np(
            tf_buffer=tf_buffer,
            parent=adjustment.header.frame_id,
            child=adjustment.child_frame_id,
            adjustment=numpify(adjustment.transform),
        )

    def adjust_transform_np(
        self,
        tf_buffer: Buffer,
        parent: str,
        child: str,
        adjustment: npt.NDArray[np.float64],
        relative_to: str | None = None,
    ) -> None:
        """Apply 4x4 adjustment to the transform. This is useful for calibration.

        :param tf_buffer: The buffer to look up the current transform
        :param parent: The parent frame of the frame to move
        :param child: The child frame to move
        :param adjustment: The 4x4 adjustment matrix to apply
        :param relative_to: The frame that 'adjustment' is in. By default, this is in
            'child' space. For example, if the adjustment is in 'frame_c' space, and is
            == np.eye(4), then the child frame will be moved to the same position as
            'frame_c'.
        """
        current_transform = numpify(
            tf_buffer.lookup_transform(
                source_frame=relative_to or child, target_frame=parent, time=Time()
            ).transform
        )
        adjusted_transform = adjustment @ current_transform

        logging.error(
            f"Applying adjustment to {parent}->{child} of {adjustment.tolist()}"
        )
        adjustment_msg = TransformStamped(
            header=Header(frame_id=parent),
            child_frame_id=child,
            transform=msgify(Transform, adjusted_transform),
        )
        self.update_transform.publish(adjustment_msg)

    def adjust_transform_along_axes(
        self,
        tf_buffer: Buffer,
        parent: str,
        child: str,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        relative_to: str | None = None,
    ) -> None:
        adjustment = np.eye(4)
        adjustment[0, 3] = x
        adjustment[1, 3] = y
        adjustment[2, 3] = z
        return self.adjust_transform_np(
            tf_buffer=tf_buffer,
            parent=parent,
            child=child,
            adjustment=adjustment,
            relative_to=relative_to,
        )
