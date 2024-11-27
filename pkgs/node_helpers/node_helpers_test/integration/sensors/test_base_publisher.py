import math
from copy import deepcopy
from unittest import mock

import pytest
from builtin_interfaces.msg import Time
from node_helpers_msgs.msg import SensorExample
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from .conftest import (
    SENSOR_FRAME_ID,
    SENSOR_TOPIC,
    ExamplePublisher,
    ExamplePubNode,
    ExampleSubNode,
)


def test_publish_sensor(
    publisher_node: ExamplePubNode, subscriber_node: ExampleSubNode
) -> None:
    """A full end-to-end test of publishing and receiving values.
    Ensures:
        - Sensor value is published
        - A marker array is generated, published, and assigned IDs and a namespace
    """
    msg = SensorExample(
        header=Header(
            frame_id=publisher_node.publisher.frame_id, stamp=Time(sec=1, nanosec=1)
        ),
        value=5,
    )
    publisher_node.publisher.publish_sensor(msg)

    # Verify the received message
    sensor_msg_received = subscriber_node.subscriber.get(timeout=5)
    assert sensor_msg_received == msg

    # Verify the visualization messages got generated and published as expected
    markers: list[Marker] = subscriber_node.marker_queue.get(timeout=5).markers
    assert len(markers) == 3
    assert markers[0].action == Marker.DELETEALL
    for index, marker in enumerate(markers[1:]):
        assert marker.header == msg.header, "Markers should share the sensor header"
        assert marker.id == index + 1


def test_publish_sensor_sanitization(publisher_node: ExamplePubNode) -> None:
    """Publishing should check the header for validity"""
    working_msg = SensorExample(
        header=Header(
            frame_id=publisher_node.publisher.frame_id, stamp=Time(sec=1, nanosec=1)
        ),
        value=5,
    )

    # This should work
    publisher_node.publisher.publish_sensor(working_msg)

    # Test publishing a header that is not relevant to the sensor
    bad_frame_id = deepcopy(working_msg)
    bad_frame_id.header.frame_id = "not the assigned frame id"
    with pytest.raises(ValueError):
        publisher_node.publisher.publish_sensor(bad_frame_id)

    # Test publishing an unset timestamp
    bad_stamp = deepcopy(working_msg)
    bad_stamp.header.stamp = Time()
    with pytest.raises(ValueError):
        publisher_node.publisher.publish_sensor(bad_stamp)

    # Validate that markers with pre-assigned headers aren't allowed
    with mock.patch.object(publisher_node.publisher, "to_rviz_msg") as to_rviz_msg:
        to_rviz_msg.return_value = [Marker(header=Header(frame_id="some preset frame"))]

        with pytest.raises(ValueError):
            publisher_node.publisher.publish_sensor(working_msg)


def test_publish_value(publisher_node: ExamplePubNode) -> None:
    """Test that publishing a value uses a current timestamp with the requisite
    frame_id"""

    # Test publishing without an integer
    expected = 1231
    with mock.patch.object(
        publisher_node.publisher._sensor_publisher, "publish"
    ) as ros_publish:
        publisher_node.publisher.publish_value(expected)

    # Verify the frame ID and a timestamp was assigned
    assert ros_publish.call_count == 1
    called_with: SensorExample = ros_publish.call_args[0][0]
    assert called_with.header.frame_id == publisher_node.publisher.frame_id
    assert called_with.header.stamp.sec != 0


def test_throttling_functionality_has_ttl_caches_assigned(
    publisher_node: ExamplePubNode, subscriber_node: ExampleSubNode
) -> None:
    """Verify that throttled visualization / sensors have ttl caches applied"""
    # Create a throttled visualizer
    throttled_publisher = ExamplePublisher(
        node=publisher_node,
        msg_type=SensorExample,
        parameters=ExamplePublisher.Parameters(
            vis_publishing_max_hz=12.0,
            sensor_publishing_max_hz=6.0,
            frame_id=SENSOR_FRAME_ID,
            sensor_topic=SENSOR_TOPIC,
        ),
    )

    throttled_vis_publish_fn = throttled_publisher._publish_rviz_markers
    normal_vis_publish_fn = publisher_node.publisher._publish_rviz_markers
    assert math.isclose(throttled_vis_publish_fn.cache.ttl_seconds, 1 / 12)  # type: ignore
    assert not hasattr(normal_vis_publish_fn, "cache")

    throttled_sensor_publish_fn = throttled_publisher.publish_sensor
    normal_sensor_publish_fn = publisher_node.publisher.publish_sensor
    assert math.isclose(throttled_sensor_publish_fn.cache.ttl_seconds, 1 / 6)  # type: ignore
    assert not hasattr(normal_sensor_publish_fn, "cache")
