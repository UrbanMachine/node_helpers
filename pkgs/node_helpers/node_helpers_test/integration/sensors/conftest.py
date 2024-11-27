from collections.abc import Generator
from typing import Any

import pytest
from node_helpers.nodes import HelpfulNode
from node_helpers.sensors import BaseSensorBuffer, BaseSensorPublisher
from node_helpers.sensors.base_publisher import DEFAULT_SENSORS_VIS_TOPIC, SENSOR_MSG
from node_helpers.testing import NodeForTesting, set_up_node
from node_helpers_msgs.msg import SensorExample
from rclpy.qos import qos_profile_services_default
from visualization_msgs.msg import Marker, MarkerArray

SENSOR_FRAME_ID = "cool-frame"
SENSOR_TOPIC = "/example_sensor"


class ExamplePublisher(
    BaseSensorPublisher[SensorExample, int, BaseSensorPublisher.Parameters]
):
    def to_rviz_msg(self, msg: SENSOR_MSG) -> list[Marker]:
        return [Marker(), Marker()]


class ExampleBuffer(BaseSensorBuffer[SensorExample]):
    pass


class ExamplePubNode(HelpfulNode):
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__("publisher", *args, **kwargs)
        self.publisher = ExamplePublisher(
            node=self,
            msg_type=SensorExample,
            parameters=ExamplePublisher.Parameters(
                frame_id=SENSOR_FRAME_ID, sensor_topic=SENSOR_TOPIC
            ),
            sensor_qos=qos_profile_services_default,
            vis_qos=qos_profile_services_default,
        )


class ExampleSubNode(NodeForTesting):
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__("subscriber", *args, **kwargs)
        self.subscriber = ExampleBuffer(self, SensorExample, SENSOR_TOPIC)
        self.marker_queue = self.create_queue_subscription(
            MarkerArray, DEFAULT_SENSORS_VIS_TOPIC
        )


@pytest.fixture()
def publisher_node() -> Generator[ExamplePubNode, None, None]:
    yield from set_up_node(
        node_class=ExamplePubNode, namespace="publish", node_name="publisher"
    )


@pytest.fixture()
def subscriber_node() -> Generator[ExampleSubNode, None, None]:
    yield from set_up_node(
        node_class=ExampleSubNode, namespace="subscribe", node_name="subscriber"
    )
