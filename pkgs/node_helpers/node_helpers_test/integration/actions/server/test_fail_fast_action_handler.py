from collections.abc import Generator
from threading import Event
from typing import Any

import pytest
from node_helpers import futures
from node_helpers.actions.server import (
    FailFastActionHandler,
    SynchronousActionCalledInParallel,
)
from node_helpers.nodes import HelpfulNode
from node_helpers.robust_rpc import RobustRPCException
from node_helpers.testing import set_up_node
from node_helpers_msgs.action import RobustActionExample
from node_helpers_msgs.action._robust_action_example import (
    RobustActionExample_GetResult_Response,
)
from rclpy.action.server import ServerGoalHandle

from .utils import ActionWorkerForTesting


class ExampleActionHandler(
    FailFastActionHandler[
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
        )
        self.done_event = Event()

    def create_worker(self, goal_handle: ServerGoalHandle) -> ActionWorkerForTesting:
        return ActionWorkerForTesting(
            goal_handle=goal_handle, done_event=self.done_event
        )


class ExampleActionServer(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("example_action_server", **kwargs)
        self.handler = ExampleActionHandler(self)


class ExampleActionClient(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("example_action_client", **kwargs)
        self.client = self.create_robust_action_client(
            RobustActionExample, "example_action"
        )


@pytest.fixture()
def example_action_server() -> Generator[ExampleActionServer, None, None]:
    yield from set_up_node(
        ExampleActionServer,
        "action_handler",
        "example_action_server",
        multi_threaded=True,
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


def test_basic_usage(
    example_action_client: ExampleActionClient,
    example_action_server: ExampleActionServer,
) -> None:
    """Test that while an action is being run, any other attempts to run it result in
    the expected exception."""

    first_request, _ = futures.wait_for_send_goal(
        example_action_client.client, RobustActionExample.Goal()
    )
    second_request, _ = futures.wait_for_send_goal(
        example_action_client.client, RobustActionExample.Goal()
    )

    # Ensure the second action runs into an exception
    with pytest.raises(RobustRPCException.like(SynchronousActionCalledInParallel)):
        futures.wait_for_future(second_request, object, timeout=10)

    # Ensure the first request is still active
    assert not first_request.done()
    assert second_request.done()

    # Allow the first request to finish
    example_action_server.handler.done_event.set()
    futures.wait_for_future(
        first_request, RobustActionExample_GetResult_Response, timeout=10
    )

    # Validate a third request can still go through without exceptions
    third_request, _ = futures.wait_for_send_goal(
        example_action_client.client, RobustActionExample.Goal()
    )
    futures.wait_for_future(
        third_request, RobustActionExample_GetResult_Response, timeout=10
    )
