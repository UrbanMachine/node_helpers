from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class ConstantStaticTransformBroadcaster:
    """This object sets up a ros timer to repeatedly re-broadcast a static transform.

    It's an analog of StaticTransformBroadcaster, but it's designed to periodically
    re-broadcast the same transform, rather than sending it once. This is after some
    observations that QoS latching behavior wasn't 100% reliable in production...
    """

    def __init__(
        self,
        node: Node,
        initial_transform: TransformStamped | None = None,
        publish_seconds: float = 1,
    ):
        """
        :param node: The node to create the timer and publish with
        :param initial_transform: Optional, the transform to publish initially
        :param publish_seconds: How many seconds inbetween repeat publishes
        """
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_tf = node.create_publisher(TFMessage, "/tf_static", qos)

        self._net_message = TFMessage(transforms=[initial_transform])
        node.create_timer(
            publish_seconds,
            callback=self._publish_transform,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _publish_transform(self) -> None:
        self.pub_tf.publish(self._net_message)

    def set_transform(self, transform: TransformStamped) -> None:
        """Update the transform that is constantly being broadcasted"""
        self._net_message.transforms = [transform]
        self._publish_transform()
