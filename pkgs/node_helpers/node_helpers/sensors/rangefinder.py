from geometry_msgs.msg import Point, Pose, Vector3
from node_helpers_msgs.msg import RangefinderReading
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_services_default
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from node_helpers import markers

from .base_buffer import BaseSensorBuffer
from .base_publisher import BaseSensorPublisher


class RangefinderPublisher(
    BaseSensorPublisher[RangefinderReading, Vector3, "RangefinderPublisher.Parameters"]
):
    """This binary signal sensor will publish only when the state changes.

    The QOS is reliable, because the sensor only publishes once per state change.
    """

    class Parameters(BaseSensorPublisher.Parameters):
        vis_publishing_max_hz: float = 10.0
        """The Rangefinder sets a default throttle for visualization, since most
        rangefinders operate as a stream of data (as opposed to binary sensors), so it
        makes sense to not waste rviz resources by publishing at a high rate."""

        sensor_publishing_max_hz: float = 20.0
        """The Rangefinder publishes on every value received instead of on value change,
        so we throttle the sensor publishing rate by default
        to avoid gumming up the ROS graph too much."""

    def __init__(self, node: Node, parameters: Parameters, qos: QoSProfile):
        super().__init__(
            node=node,
            msg_type=RangefinderReading,
            parameters=parameters,
            sensor_qos=qos,
        )

    def publish_range(self, value: float) -> Vector3:
        """Publish a new range value, in meters."""
        self.publish_value(Vector3(z=value))

    def to_rviz_msg(self, msg: RangefinderReading) -> list[Marker]:
        """Create an arrow that reflects the rangefinder reading."""
        color = (
            ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0)
            if msg.value
            else ColorRGBA(r=0.4, g=0.2, b=0.2, a=1.0)
        )
        return [
            # Mark frame_locked=True to fix flickering effect in rviz.
            # This essentially lets rviz render the marker even if it hasn't yet
            # received up-to-date TF information.
            markers.create_point_to_point_arrow_marker(
                shaft_diameter=0.01,
                head_diameter=0.02,
                head_length=0.03,
                base_point=(0.0, 0.0, 0.0),
                head_point=msg.value,
                color=color,
                frame_locked=True,
            ),
            markers.create_floating_text(
                text=f"{msg.value.z:.3f}",
                text_height=0.05,
                color=color,
                # Offset the text from the base of the sensor frame
                pose=Pose(position=Point(x=0.05, y=0.05, z=0.05)),
                frame_locked=True,
            ),
        ]


class RangefinderBuffer(BaseSensorBuffer[RangefinderReading]):
    def __init__(
        self, node: Node, sensor_topic: str, callback_group: CallbackGroup | None = None
    ):
        super().__init__(
            node=node,
            msg_type=RangefinderReading,
            sensor_topic=sensor_topic,
            sensor_qos=qos_profile_services_default,
            callback_group=callback_group,
        )
