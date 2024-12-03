from copy import deepcopy
from queue import Empty, Queue
from threading import Event

import pytest
from builtin_interfaces.msg import Time
from node_helpers.testing import DynamicContextThread
from node_helpers_msgs.msg import SensorExample
from std_msgs.msg import Header

from .conftest import SENSOR_FRAME_ID, ExamplePubNode, ExampleSubNode

EXAMPLE_MSG = SensorExample(
    header=Header(frame_id=SENSOR_FRAME_ID, stamp=Time(sec=1, nanosec=1)), value=5
)


def test_on_value_change(
    publisher_node: ExamplePubNode, subscriber_node: ExampleSubNode
) -> None:
    """Test the ON_VALUE_CHANGE callback only calls when the value changes"""
    msg = deepcopy(EXAMPLE_MSG)

    # Subscribe a queue to all changes in value
    on_value_change: Queue[int] = Queue()
    subscriber_node.subscriber.on_value_change.subscribe(on_value_change.put)

    # The first publish should always count as a value "change"
    publisher_node.publisher.publish_sensor(msg)
    received_msg = on_value_change.get(timeout=0.5)
    assert received_msg == msg

    # Publish the same value, and expect no changes
    publisher_node.publisher.publish_sensor(msg)
    with pytest.raises(Empty):
        on_value_change.get(timeout=0.5)

    # Change the value and validate the callback was called again
    msg.value = 1337
    publisher_node.publisher.publish_sensor(msg)
    received_msg = on_value_change.get(timeout=0.5)
    assert received_msg == msg
    assert on_value_change.qsize() == 0


def test_on_receive(
    publisher_node: ExamplePubNode, subscriber_node: ExampleSubNode
) -> None:
    """Test the ON_RECEIVE callback calls for each single publish"""
    msg = deepcopy(EXAMPLE_MSG)

    # Subscribe a queue to all publishes, regardless of whether it's changed
    on_receive: Queue[int] = Queue()
    subscriber_node.subscriber.on_receive.subscribe(on_receive.put)

    # Try publishing and receiving a few times
    for _ in range(3):
        publisher_node.publisher.publish_sensor(msg)
        received_msg = on_receive.get(timeout=0.5)
        assert received_msg == msg

    assert on_receive.qsize() == 0


def test_get_timeouts(
    publisher_node: ExamplePubNode, subscriber_node: ExampleSubNode
) -> None:
    """Test the 'get' timeout functionality"""
    # When no value has ever been received
    with pytest.raises(TimeoutError):
        subscriber_node.subscriber.get(timeout=0.1)

    # When there is some value, but no 'after' parameter is set, the message should be
    # immediately returned
    msg = deepcopy(EXAMPLE_MSG)
    subscriber_node.subscriber._latest_reading = msg
    assert subscriber_node.subscriber.get(timeout=0.0) == msg

    # If a message is requested with an "after" that is greater than the current message
    # then it should block until timeout
    with pytest.raises(TimeoutError):
        subscriber_node.subscriber.get(after=Time(sec=99999999), timeout=0.1)
    # Make sure there's no bugs when timeout=0.0
    with pytest.raises(TimeoutError):
        subscriber_node.subscriber.get(after=Time(sec=99999999), timeout=0.0)

    # If a message is requested with an "after" that is < current message, it should
    # return the last message immediately, regardless of timeout
    assert (
        subscriber_node.subscriber.get(after=Time(sec=0, nanosec=1), timeout=0.0) == msg
    )


def test_get_after(
    publisher_node: ExamplePubNode, subscriber_node: ExampleSubNode
) -> None:
    """Test the 'get' functionality when requesting a timestamp greater than current"""
    on_getting = Event()
    on_gotten = Event()
    after_stamp = Time(sec=3, nanosec=0)
    msg = deepcopy(EXAMPLE_MSG)
    after_msg = deepcopy(msg)
    after_msg.header.stamp = after_stamp

    def wait_for_get() -> None:
        on_getting.set()
        assert subscriber_node.subscriber.get(after=after_stamp) == after_msg
        on_gotten.set()

    with DynamicContextThread(target=wait_for_get):
        # Wait for the system to be 'getting'
        assert on_getting.wait(10)
        # The "get" should subscribe to changes
        while len(subscriber_node.subscriber.on_receive._subscribers) == 0:
            pass
        assert not on_gotten.is_set()

        # Now publish a few changes that are not past the requested number
        for i in range(3):
            msg.header.stamp.nanosec = i
            publisher_node.publisher.publish_sensor(msg)
        assert not on_gotten.wait(0.5)

        # Now "publish" a change with an 'after' that is past the requested number
        publisher_node.publisher.publish_sensor(after_msg)
        assert on_gotten.wait(0.5)


def test_reject_out_of_order_messages(subscriber_node: ExampleSubNode) -> None:
    # Create two messages with different timestamps
    newer_msg = deepcopy(EXAMPLE_MSG)
    newer_msg.header.stamp = Time(sec=2, nanosec=0)

    older_msg = deepcopy(EXAMPLE_MSG)
    older_msg.header.stamp = Time(sec=1, nanosec=0)

    # Publish a newer message and then an older one
    subscriber_node.subscriber._on_receive(newer_msg)
    subscriber_node.subscriber._on_receive(older_msg)

    # Check if the older message was rejected
    assert subscriber_node.subscriber._latest_reading == newer_msg
