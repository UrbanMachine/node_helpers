import asyncio
import queue
from collections.abc import Generator
from typing import Any

import pytest
from node_helpers.async_tools import AsyncAdapter
from node_helpers.futures import wait_for_future
from node_helpers.nodes import HelpfulNode
from node_helpers.testing import set_up_node
from node_helpers_msgs.msg import SensorExample
from node_helpers_msgs.srv import RobustServiceExample
from rclpy.qos import qos_profile_services_default


class ExampleAsyncNode(HelpfulNode):
    """A node that uses async callbacks for topic subscriptions and services"""

    def __init__(self, **kwargs: Any):
        super().__init__("example_async_node", **kwargs)

        self.async_adapter = AsyncAdapter(self)

        self.message_received: queue.Queue[SensorExample] = queue.Queue()
        self.service_received: queue.Queue[RobustServiceExample.Request] = queue.Queue()

        self.create_subscription(
            SensorExample,
            "sensor",
            self.async_adapter.adapt(self.on_message),
            qos_profile=qos_profile_services_default,
        )

        self.create_service(
            RobustServiceExample, "service", self.async_adapter.adapt(self.on_service)
        )

    async def on_message(self, message: SensorExample) -> None:
        await asyncio.sleep(0)
        self.message_received.put(message)

    async def on_service(
        self,
        request: RobustServiceExample.Request,
        response: RobustServiceExample.Response,
    ) -> RobustServiceExample.Response:
        await asyncio.sleep(0)
        self.service_received.put(request)
        return response


class ExampleAsyncNodeClient(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("example_async_node_client", **kwargs)

        self.send_message = self.create_publisher(
            SensorExample, "sensor", qos_profile=qos_profile_services_default
        )

        self.call_service = self.create_client(RobustServiceExample, "service")


@pytest.fixture()
def example_async_node() -> Generator[ExampleAsyncNode, None, None]:
    yield from set_up_node(
        node_class=ExampleAsyncNode,
        namespace="async_adapter",
        node_name="example_async_node",
        multi_threaded=True,
    )


@pytest.fixture()
def example_async_node_client() -> Generator[ExampleAsyncNodeClient, None, None]:
    yield from set_up_node(
        node_class=ExampleAsyncNodeClient,
        namespace="async_adapter",
        node_name="example_async_node_client",
    )


def test_callback_adapter(
    example_async_node: ExampleAsyncNode,
    example_async_node_client: ExampleAsyncNodeClient,
) -> None:
    """Tests that the callback adapter works for topics and services"""

    # Topic subscription
    example_async_node_client.send_message.publish(SensorExample(value=1))
    message = example_async_node.message_received.get(timeout=5.0)
    assert message.value == 1

    # Service
    future = example_async_node_client.call_service.call_async(
        RobustServiceExample.Request()
    )
    example_async_node.service_received.get(timeout=5.0)
    wait_for_future(future, RobustServiceExample.Response, timeout=5.0)
