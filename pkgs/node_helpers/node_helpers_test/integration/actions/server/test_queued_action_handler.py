import queue
from collections.abc import Generator
from threading import Event
from typing import Any

import pytest
from node_helpers.actions.server import QueuedActionHandler
from node_helpers.futures import wait_for_future
from node_helpers.nodes import HelpfulNode
from node_helpers.testing import set_up_node
from node_helpers_msgs.action import RobustActionExample
from node_helpers_msgs.action._robust_action_example import (
    RobustActionExample_GetResult_Response,
)
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle
from rclpy.task import Future

from .utils import ActionWorkerForTesting


class ExampleActionHandler(
    QueuedActionHandler[
        RobustActionExample.Goal,
        RobustActionExample.Feedback,
        RobustActionExample.Result,
    ]
):
    def __init__(self, node: HelpfulNode, done_events: "queue.Queue[Event]"):
        super().__init__(
            node=node,
            action_name="example_action",
            action_type=RobustActionExample,
        )
        self.done_events = done_events

    def create_worker(self, goal_handle: ServerGoalHandle) -> ActionWorkerForTesting:
        done_event = Event()
        self.done_events.put(done_event)
        return ActionWorkerForTesting(goal_handle=goal_handle, done_event=done_event)


class ExampleActionServer(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("example_action_server", **kwargs)
        self.done_events: "queue.Queue[Event]" = queue.Queue()
        self.handler = ExampleActionHandler(self, self.done_events)


@pytest.fixture()
def example_action_server() -> Generator[ExampleActionServer, None, None]:
    yield from set_up_node(
        ExampleActionServer,
        "action_handler",
        "example_action_server",
        multi_threaded=True,
    )


class ExampleActionClient(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("example_action_client", **kwargs)
        self.client = self.create_robust_action_client(
            RobustActionExample, "example_action"
        )


@pytest.fixture()
def example_action_client(
    example_action_server: ExampleActionServer,
) -> Generator[ExampleActionClient, None, None]:
    yield from set_up_node(
        ExampleActionClient,
        "action_handler",
        "example_action_client",
    )


def test_queued_work(
    example_action_client: ExampleActionClient,
    example_action_server: ExampleActionServer,
) -> None:
    """Tests that the action server responds to multiple requests one-by-one"""
    first_request: Future = wait_for_future(
        example_action_client.client.send_goal_async(RobustActionExample.Goal()),
        ClientGoalHandle,
        timeout=10,
    ).get_result_async()
    second_request: Future = wait_for_future(
        example_action_client.client.send_goal_async(RobustActionExample.Goal()),
        ClientGoalHandle,
        timeout=10,
    ).get_result_async()

    # Ensure that both actions are waiting
    assert not first_request.done()
    assert not second_request.done()

    # Let the first action go through
    example_action_server.done_events.get().set()
    wait_for_future(first_request, RobustActionExample_GetResult_Response, timeout=10)
    assert first_request.done()
    assert not second_request.done()

    # Let the second action go through
    example_action_server.done_events.get().set()
    wait_for_future(second_request, RobustActionExample_GetResult_Response, timeout=10)
    assert second_request.done()
