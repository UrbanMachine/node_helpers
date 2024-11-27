from typing import TypeVar

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Vector3
from node_helpers_msgs.msg import BinaryReading, RangefinderReading
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from ..pubsub import Topic
from .base_buffer import BaseSensorBuffer
from .base_publisher import BaseSensorPublisher
from .rangefinder import RangefinderBuffer

PARAMETERS = TypeVar("PARAMETERS", bound=BaseSensorPublisher.Parameters)


def _to_rviz_msg(msg: BinaryReading) -> list[Marker]:
    """Create a cylinder that turns green when the sensor is triggered"""
    color = (
        ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0)
        if msg.value
        else ColorRGBA(r=0.4, g=0.2, b=0.2, a=1.0)
    )
    return [
        Marker(
            type=Marker.CYLINDER,
            scale=Vector3(x=0.05, y=0.05, z=0.1),
            color=color,
            lifetime=Duration(sec=60 * 60),
            frame_locked=True,
        )
    ]


class BinarySensor(
    BaseSensorPublisher[
        BinaryReading, bool, "BinarySensorFromFieldPublisher.Parameters"
    ]
):
    def __init__(self, node: Node, parameters: BaseSensorPublisher.Parameters):
        super().__init__(
            node=node,
            msg_type=BinaryReading,
            parameters=parameters,
            sensor_qos=qos_profile_action_status_default,
        )

    def to_rviz_msg(self, msg: BinaryReading) -> list[Marker]:
        return _to_rviz_msg(msg)


class BinarySensorFromRangeFinder(
    BaseSensorPublisher[BinaryReading, bool, "BinarySensorFromRangeFinder.Parameters"]
):
    class Parameters(BaseSensorPublisher.Parameters):
        threshold: float
        """A threshold in meters. If the rangefinder reading is below this value, the
        sensor will be triggered. Use 'invert' to flip this behavior.
        """

        inverted: bool = False
        """If True, the sensor will be triggered when the rangefinder reading is above
        the threshold."""

        vis_publishing_max_hz: float = 10.0
        """The Rangefinder sets a default throttle for visualization, since most
        rangefinders operate as a stream of data (as opposed to binary sensors), so it
        makes sense to not waste rviz resources by publishing at a high rate."""

    def __init__(
        self,
        node: Node,
        rangefinder: RangefinderBuffer,
        parameters: Parameters,
    ):
        super().__init__(
            node=node,
            msg_type=BinaryReading,
            parameters=parameters,
            sensor_qos=qos_profile_action_status_default,
        )
        rangefinder.on_value_change.subscribe(self._on_rangefinder_change)

    def _on_rangefinder_change(self, reading: RangefinderReading) -> None:
        # Check if the sensor should be triggered
        value = reading.value.z < self._params.threshold
        if self._params.inverted:
            value = not value

        self.publish_value(value, stamp=reading.header.stamp)

    def to_rviz_msg(self, msg: BinaryReading) -> list[Marker]:
        return _to_rviz_msg(msg)


class BinarySensorBuffer(BaseSensorBuffer[BinaryReading]):
    def __init__(
        self, node: Node, sensor_topic: str, callback_group: CallbackGroup | None = None
    ):
        super().__init__(
            node=node,
            msg_type=BinaryReading,
            sensor_topic=sensor_topic,
            sensor_qos=qos_profile_action_status_default,
            callback_group=callback_group,
        )

        self.on_rising_edge = Topic[BinaryReading]()
        """Called when the sensor transitions from False to True"""

        self.on_falling_edge = Topic[BinaryReading]()
        """Called when the sensor transitions from True to False"""

    def _on_receive(self, msg: BinaryReading) -> None:
        """Called whenever a sensor reading is received"""
        if self._latest_reading is not None:
            if not self._latest_reading.value and msg.value:
                self.on_rising_edge.publish(msg)
            elif self._latest_reading.value and not msg.value:
                self.on_falling_edge.publish(msg)

        return super()._on_receive(msg)
