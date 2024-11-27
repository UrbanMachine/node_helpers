from collections.abc import Generator
from copy import deepcopy
from queue import Queue
from threading import Event
from typing import Any
from uuid import UUID

import pytest
from node_helpers.actions.server import ActionCallMetric, ActionHandler
from node_helpers.futures import wait_for_future
from node_helpers.nodes import HelpfulNode
from node_helpers.robust_rpc import RobustRPCException
from node_helpers.testing import NodeForTesting, set_up_node
from node_helpers_msgs.action import RobustActionExample
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from .utils import ActionWorkerForTesting, CoolError


class ExampleActionHandler(
    ActionHandler[
        RobustActionExample.Goal,
        RobustActionExample.Feedback,
        RobustActionExample.Result,
    ]
):
    def __init__(self, node: HelpfulNode):
        super().__init__(
            node=node,
            action_name="example_action",
            action_type=RobustActionExample,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.raise_exception = False
        self.on_exception_called = Event()
        self.done_event: Event | None = None

    def create_worker(self, goal_handle: ServerGoalHandle) -> ActionWorkerForTesting:
        return ActionWorkerForTesting(
            goal_handle,
            done_event=self.done_event,
            raise_exception=self.raise_exception,
            on_exception_called=self.on_exception_called,
        )


class ExampleActionServer(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("example_action_server", **kwargs)
        self.handler = ExampleActionHandler(self)


@pytest.fixture()
def example_action_server() -> Generator[ExampleActionServer, None, None]:
    yield from set_up_node(
        ExampleActionServer,
        "action_handler",
        "example_action_server",
        multi_threaded=True,
    )


class ExampleActionClient(NodeForTesting):
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
        multi_threaded=True,
    )


def test_basic_operation(
    example_action_server: ExampleActionServer,
    example_action_client: ExampleActionClient,
) -> None:
    """Tests that the action server responds properly to a single goal"""
    result = example_action_client.client.send_goal(RobustActionExample.Goal())
    assert result.result.data == "success"


def test_on_exception(
    example_action_client: ExampleActionClient,
    example_action_server: ExampleActionServer,
) -> None:
    """Tests that the action server reports an error to the client when an exception is
    raised
    """
    example_action_server.handler.raise_exception = True

    with pytest.raises(RobustRPCException.like(CoolError)):
        example_action_client.client.send_goal(RobustActionExample.Goal())

    assert example_action_server.handler.on_exception_called.is_set()


@pytest.mark.parametrize("expected_result", ["success", "error", "canceled"])
def test_report_metric(
    example_action_client: ExampleActionClient,
    example_action_server: ExampleActionServer,
    expected_result: str,
) -> None:
    """Tests that the action server sends metrics reporting information on the action
    start time and final result.
    """
    publish_queue: Queue[ActionCallMetric] = Queue()

    def on_metric(m: ActionCallMetric) -> None:
        # The action server modifies metrics in-place, so we need to copy them to keep
        # their original state
        publish_queue.put(deepcopy(m))

    example_action_server.handler._metrics_callback = on_metric

    # Set up the action to either error out or allow cancellation, when appropriate
    if expected_result == "error":
        example_action_server.handler.raise_exception = True
    elif expected_result == "canceled":
        # Make the action wait on an event so we have the chance to cancel it
        example_action_server.handler.done_event = Event()

    goal_handle = wait_for_future(
        example_action_client.client.send_goal_async(RobustActionExample.Goal()),
        ClientGoalHandle,
        timeout=10,
    )

    if expected_result == "canceled":
        goal_handle.cancel_goal()

    if expected_result == "error":
        with pytest.raises(RobustRPCException.like(CoolError)):
            goal_handle.get_result()
    else:
        goal_handle.get_result()

    assert publish_queue.qsize() == 2

    # Validate the first publish includes a goal_id/name/namespace
    start_metric = publish_queue.get_nowait()
    assert start_metric.result == "in_progress"

    # Validate the
    final_metric = publish_queue.get_nowait()
    elapsed = final_metric.elapsed
    assert isinstance(elapsed, float)
    assert elapsed > 0
    assert final_metric.result == expected_result

    # Validate fields that are common across both metrics
    expected_uuid = UUID(bytes=bytes(goal_handle.goal_id.uuid))
    for metric in (start_metric, final_metric):
        assert metric.action_name == "example_action"
        assert metric.node_namespace == "/action_handler"

        # Validate both metrics share the same goal_id
        assert metric.goal_id == expected_uuid
