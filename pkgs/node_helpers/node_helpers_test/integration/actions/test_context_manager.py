from collections.abc import Generator
from itertools import cycle
from queue import Queue
from typing import Any

import pytest
from node_helpers.actions.context_manager import ActionContextManager
from node_helpers.nodes import HelpfulNode
from node_helpers.robust_rpc import RobustRPCException
from node_helpers.testing import (
    ActionServerCallback,
    ConfigurableServiceCallback,
    set_up_node,
)
from node_helpers_msgs.action import RobustActionExample
from node_helpers_msgs.action._robust_action_example import RobustActionExample_Feedback
from rclpy.action.server import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup


class ExampleServerNode(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("example_server_node", **kwargs)

        # Create "action_a", a very basic action that calls the action_callback
        self.action_a_callback = ActionServerCallback(
            return_values=cycle([RobustActionExample.Result()]),
            feedback_values=cycle([RobustActionExample.Feedback()]),
        )
        self.action_a_callback.allow_publish_feedback.set()
        self.create_robust_action_server(
            RobustActionExample,
            "/test_context_manager/action_a",
            self.action_a_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=lambda *args: CancelResponse.ACCEPT,
        )

        # Create "action_b", a very basic action that calls the action_callback
        self.action_b_callback = ActionServerCallback(
            return_values=cycle([RobustActionExample.Result()]),
            feedback_values=cycle([RobustActionExample.Feedback()]),
        )
        self.action_b_callback.allow_publish_feedback.set()
        self.create_robust_action_server(
            RobustActionExample,
            "/test_context_manager/action_b",
            self.action_b_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=lambda *args: CancelResponse.ACCEPT,
        )


@pytest.fixture()
def example_server_node() -> Generator[ExampleServerNode, None, None]:
    yield from set_up_node(
        node_class=ExampleServerNode,
        namespace="test_context_manager",
        node_name="example_server_node",
        multi_threaded=True,
    )


class ExampleClientNode(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("example_client_node", **kwargs)

        self.action_a_client = self.create_robust_action_client(
            RobustActionExample, "/test_context_manager/action_a"
        )
        self.action_b_client = self.create_robust_action_client(
            RobustActionExample, "/test_context_manager/action_b"
        )


@pytest.fixture()
def example_client_node() -> Generator[ExampleClientNode, None, None]:
    yield from set_up_node(
        node_class=ExampleClientNode,
        namespace="test_context_manager",
        node_name="example_client_node",
        multi_threaded=True,
    )


def test_basic_operation(
    example_server_node: ExampleServerNode, example_client_node: ExampleClientNode
) -> None:
    """Tests that the ActionContextManager interacts properly with the action"""
    action_callback = example_server_node.action_a_callback
    assert not action_callback.on_action_started.is_set()

    with ActionContextManager(
        example_client_node.action_a_client, RobustActionExample.Goal(), timeout=10
    ):
        # Ensure that the action has started and no cancellation has been requested
        assert action_callback.on_action_started.is_set()
        assert not action_callback.on_cancel_requested.is_set()

        # Allow cancellation before exiting the context
        action_callback.allow_cancel.set()

    assert action_callback.call_count == 1
    assert action_callback.on_cancel_requested.is_set()


def test_async(
    example_server_node: ExampleServerNode, example_client_node: ExampleClientNode
) -> None:
    """Tests that two ActionContextManagers can be called concurrently using its async
    mode
    """

    call_a = ActionContextManager(
        example_client_node.action_a_client,
        RobustActionExample.Goal(),
        timeout=10,
        async_=True,
    )
    call_b = ActionContextManager(
        example_client_node.action_b_client,
        RobustActionExample.Goal(),
        timeout=10,
        async_=True,
    )
    callback_a = example_server_node.action_a_callback
    callback_b = example_server_node.action_b_callback

    with call_a, call_b:
        call_a.wait_for_feedback()
        call_b.wait_for_feedback()

        # Both actions should be running right now
        assert callback_a.on_action_started.is_set()
        assert callback_b.on_action_started.is_set()
        assert not callback_a.on_cancel_requested.is_set()
        assert not callback_b.on_cancel_requested.is_set()

        # Allow cancellation before exiting the context
        callback_a.allow_cancel.set()
        callback_b.allow_cancel.set()

    call_a.wait_for_cancellation()
    call_b.wait_for_cancellation()

    assert callback_a.on_cancel_requested.is_set()
    assert callback_b.on_cancel_requested.is_set()

    # Ensure that these wait_for_* methods can be called multiple times
    call_a.wait_for_feedback()
    call_a.wait_for_cancellation()


def test_check_for_exceptions(
    example_server_node: ExampleServerNode, example_client_node: ExampleClientNode
) -> None:
    """Test that 'check_for_exceptions' nonblockingly checks for exceptions"""

    action_callback = example_server_node.action_a_callback
    action_callback._return_value_iterator = ConfigurableServiceCallback(
        [RobustActionExample.Result(error_name="SomeActionError")]
    )

    assert not action_callback.on_action_started.is_set()

    # An exception should be raised when the context is exited
    expected_exception = RobustRPCException.like("SomeActionError")
    with (
        pytest.raises(expected_exception),
        ActionContextManager(
            example_client_node.action_a_client, RobustActionExample.Goal(), timeout=10
        ) as handle,
    ):
        # Ensure that the action has started and no cancellation has been requested
        assert action_callback.on_action_started.is_set()
        assert not action_callback.on_cancel_requested.is_set()

        # Allow cancellation before exiting the context
        action_callback.allow_abort.set()

    # 'check_for_exceptions' should also work
    with pytest.raises(expected_exception):
        handle.check_for_exceptions()


def test_feedback_is_passed_through(
    example_server_node: ExampleServerNode, example_client_node: ExampleClientNode
) -> None:
    feedback_queue: Queue[RobustActionExample_Feedback] = Queue()
    action_callback = example_server_node.action_a_callback

    context = ActionContextManager(
        example_client_node.action_a_client,
        RobustActionExample.Goal(),
        timeout=10,
        on_feedback=feedback_queue.put,
    )

    with context:
        # Allow cancellation before exiting the context
        action_callback.allow_cancel.set()

    assert action_callback.call_count == 1
    assert action_callback.on_cancel_requested.is_set()

    # Ensure that the feedback was passed through
    assert not feedback_queue.empty()
    assert feedback_queue.get() == RobustActionExample.Feedback()
