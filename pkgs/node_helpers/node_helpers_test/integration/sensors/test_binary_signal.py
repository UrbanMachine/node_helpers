from collections.abc import Generator
from typing import Any

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3
from node_helpers.sensors.binary_signal import BinarySensorFromRangeFinder
from node_helpers.sensors.rangefinder import RangefinderBuffer
from node_helpers.testing import NodeForTesting, set_up_node
from node_helpers_msgs.msg import BinaryReading, RangefinderReading
from std_msgs.msg import Header


class ClientNode(NodeForTesting):
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs, node_name="node")

        self.binary_queue = self.create_queue_subscription(
            BinaryReading, "binary_topic"
        )
        self.rangefinder_queue = self.create_queue_subscription(
            RangefinderReading, "rangefinder_topic"
        )
        self.rangefinder_publisher = self.create_publisher(
            RangefinderReading, "rangefinder_topic", 10
        )


@pytest.fixture()
def node() -> Generator[ClientNode, None, None]:
    yield from set_up_node(
        node_class=ClientNode,
        namespace="test_namespace",
        node_name="test_node",
        multi_threaded=True,
    )


def test_binary_sensor_from_rangefinder(node: ClientNode) -> None:
    """Test that the BinarySensorFromRangeFinder updates the binary state correctly"""
    rangefinder_buffer = RangefinderBuffer(
        node=node,
        sensor_topic="rangefinder_topic",
        callback_group=None,
    )
    parameters = BinarySensorFromRangeFinder.Parameters(
        sensor_topic="binary_topic",
        vis_topic="visualization_topic",
        frame_id="test_frame",
        threshold=1.0,
        inverted=False,
    )
    binary_sensor = BinarySensorFromRangeFinder(  # noqa: F841
        node=node,
        rangefinder=rangefinder_buffer,
        parameters=parameters,
    )

    # Simulate a rangefinder reading below the threshold
    reading_below_threshold = RangefinderReading(
        header=Header(frame_id="test_frame", stamp=Time(sec=3)),
        value=Vector3(x=0.0, y=0.0, z=0.5),
    )

    assert node.binary_queue.qsize() == 0
    node.rangefinder_publisher.publish(reading_below_threshold)

    # Validate that the binary sensor is triggered
    assert node.binary_queue.get(timeout=5.0).value is True

    # Simulate a rangefinder reading above the threshold
    reading_above_threshold = RangefinderReading(
        header=Header(frame_id="test_frame", stamp=Time(sec=3)),
        value=Vector3(x=0.0, y=0.0, z=1.5),
    )
    node.rangefinder_publisher.publish(reading_above_threshold)

    # Validate that the binary sensor is not triggered
    assert node.binary_queue.get(timeout=5.0).value is False

    # Simulate a rangefinder reading below the threshold again
    node.rangefinder_publisher.publish(reading_below_threshold)

    # Validate that the binary sensor is triggered again
    assert node.binary_queue.get(timeout=5.0).value is True
