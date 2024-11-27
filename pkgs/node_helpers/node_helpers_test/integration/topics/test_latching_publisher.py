from collections.abc import Generator
from time import sleep
from typing import Any

import pytest
from node_helpers.testing import NodeForTesting, set_up_node
from node_helpers.timing import TestingTimeout as Timeout
from node_helpers.topics.latching_publisher import LatchingPublisher
from std_msgs.msg import String


class ClientNode(NodeForTesting):
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs, node_name="node")

        self.msg_queue = self.create_queue_subscription(String, "test_topic")


@pytest.fixture()
def node() -> Generator[ClientNode, None, None]:
    yield from set_up_node(
        node_class=ClientNode,
        namespace="test_namespace",
        node_name="test_node",
        multi_threaded=True,
    )


def test_publish_message(node: ClientNode) -> None:
    """Test that the LatchingPublisher publishes messages correctly"""
    publisher = LatchingPublisher(node, String, "test_topic", republish_delay=0.1)
    test_message = String(data="Hello, world!")

    publisher(test_message)

    # Validate that the publisher publishes the message
    assert node.msg_queue.get(timeout=5.0).data == "Hello, world!"
    assert node.msg_queue.get(timeout=5.0).data == "Hello, world!"
    assert node.msg_queue.get(timeout=5.0).data == "Hello, world!"


def test_change_message(node: ClientNode) -> None:
    """Test that the LatchingPublisher updates the message correctly"""
    publisher = LatchingPublisher(node, String, "test_topic", republish_delay=0.1)
    initial_message = String(data="Initial message")
    updated_message = String(data="Updated message")

    publisher(initial_message)

    # Validate that the initial message is published
    assert node.msg_queue.get(timeout=5.0).data == "Initial message"

    # Update the message
    publisher(updated_message)

    # Wait for the republish delay and validate the updated message
    timeout = Timeout(seconds=25)
    while timeout and node.msg_queue.get(timeout=5.0).data == "Initial message":
        sleep(0.01)

    # The updated message should now be being published
    assert node.msg_queue.get(timeout=5.0).data == "Updated message"
    assert node.msg_queue.get(timeout=5.0).data == "Updated message"
    assert node.msg_queue.get(timeout=5.0).data == "Updated message"
