import logging
from collections.abc import Generator
from time import sleep
from typing import Any

import pytest
from geometry_msgs.msg import Transform, TransformStamped
from node_helpers.testing import NodeForTesting, set_up_node
from node_helpers.tf import ConstantStaticTransformBroadcaster
from node_helpers.timing import TestingTimeout as Timeout
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


class ClientNode(NodeForTesting):
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs, node_name="node")

        self.tf_queue = self.create_queue_subscription(TFMessage, "/tf_static")


@pytest.fixture()
def node() -> Generator[ClientNode, None, None]:
    yield from set_up_node(
        node_class=ClientNode,
        namespace="ayylaymayo",
        node_name="nodeynode",
        multi_threaded=True,
    )


def test_basic_operation(node: ClientNode) -> None:
    """Test that the broadcaster broadcasts regularly and allows updating the TF"""
    tf_1 = TransformStamped(transform=Transform(), header=Header(frame_id="first"))
    tf_2 = TransformStamped(transform=Transform(), header=Header(frame_id="second"))

    broadcaster = ConstantStaticTransformBroadcaster(
        node=node, publish_seconds=0.1, initial_transform=tf_1
    )

    tf_1_expected = TFMessage(transforms=[tf_1])
    tf_2_expected = TFMessage(transforms=[tf_2])

    # Validate that the publisher constantly publishes
    assert node.tf_queue.get(timeout=5.0) == tf_1_expected
    assert node.tf_queue.get(timeout=5.0) == tf_1_expected
    assert node.tf_queue.get(timeout=5.0) == tf_1_expected

    # Set a new transform
    broadcaster.set_transform(tf_2)

    # Wait until it is broadcasting the new transform
    timeout = Timeout(seconds=25)
    while timeout and node.tf_queue.get(timeout=5.0) == tf_1_expected:
        logging.error(
            f"OWO {node.tf_queue.qsize()=} "
            f"{broadcaster._transform.header.frame_id=} "
            f"{node.tf_queue.get(timeout=5.0).transforms[0].header.frame_id=}"
            f"{tf_1_expected.transforms[0].header.frame_id=}"
        )
        sleep(0.01)

    # The new transform should now be being published
    assert node.tf_queue.get(timeout=5.0) == tf_2_expected
    assert node.tf_queue.get(timeout=5.0) == tf_2_expected
    assert node.tf_queue.get(timeout=5.0) == tf_2_expected
