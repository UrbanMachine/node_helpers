from threading import Event

import pytest
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from node_helpers.futures import wait_for_future
from node_helpers.robust_rpc import RobustRPCException
from node_helpers.robust_rpc.errors import ActionCancellationRejected
from node_helpers_msgs.action import RobustActionExample
from node_helpers_msgs.action._robust_action_example import (
    RobustActionExample_GetResult_Response,
)

from ..conftest import RobustCaller, RobustImplementer


@pytest.mark.parametrize("wait_for_action", ((True,), (False,)))
def test_cancellation(
    threaded_implementer: RobustImplementer, caller: RobustCaller, wait_for_action: bool
) -> None:
    """Basic test that the action is cancelled when the context ends"""

    threaded_implementer.action_continue_event = Event()

    with caller.action_client.send_goal_as_context(
        RobustActionExample.Goal(), timeout=15
    ) as handle:
        assert handle.accepted
        if wait_for_action:
            # Wait for the action to be reached
            threaded_implementer.action_reached_event.wait(timeout=15)

    assert not threaded_implementer.action_continue_event.is_set()
    assert handle.status == GoalStatus.STATUS_CANCELED
    assert threaded_implementer.action_calls == 1

    # Verify that calling get_result twice (after the context already did) is fine, and
    # that it yields expected data.
    response: RobustActionExample.Result = handle.get_result().result
    assert response.data == RobustImplementer.EXPECTED_CANCELLED_DATA


def test_cancelling_uncancellable_action_raises_exception(
    threaded_implementer: RobustImplementer, caller: RobustCaller
) -> None:
    """If an action server rejects cancellation, an exception should be raised"""
    threaded_implementer.action_continue_event = Event()
    threaded_implementer.action_cancellable = False

    # Happy path (cancel = False and goal can finish)
    with caller.action_client.send_goal_as_context(
        RobustActionExample.Goal(), timeout=15, cancel=False
    ) as handle:
        threaded_implementer.action_continue_event.set()

    assert handle.status == GoalStatus.STATUS_SUCCEEDED

    # Unhappy path (cancel = True)
    threaded_implementer.action_continue_event.clear()
    with (
        pytest.raises(ActionCancellationRejected),
        caller.action_client.send_goal_as_context(
            RobustActionExample.Goal(), timeout=15, cancel=True
        ),
    ):
        pass

    # This should let the called action finish during teardown
    threaded_implementer.action_continue_event.set()


def test_no_cancellation_if_action_was_already_over(
    threaded_implementer: RobustImplementer, caller: RobustCaller
) -> None:
    """Make sure that calling get_result within the context works as expected."""
    with caller.action_client.send_goal_as_context(
        RobustActionExample.Goal(), timeout=15
    ) as handle:
        # Wait for the result of the action _before_ the end of the context
        wait_for_future(
            handle.get_result_async(),
            type_=RobustActionExample_GetResult_Response,
            timeout=15,
        )
        assert handle.status == GoalStatus.STATUS_SUCCEEDED

    # Verify no change in status after the context exits
    assert handle.status == GoalStatus.STATUS_SUCCEEDED


def test_no_cancellation_if_action_was_cancelled_in_context(
    threaded_implementer: RobustImplementer, caller: RobustCaller
) -> None:
    """Make sure that calling cancel_goal in the context also works as expected"""
    threaded_implementer.action_continue_event = Event()

    with caller.action_client.send_goal_as_context(
        RobustActionExample.Goal(), timeout=15
    ) as handle:
        # Wait for the result of the action _before_ the end of the context
        wait_for_future(
            handle.cancel_goal_async(),
            type_=CancelGoal.Response,
            timeout=15,
        )
        assert handle.status in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_CANCELING,
        )

    # Verify no change in status after the context exits
    assert handle.status == GoalStatus.STATUS_CANCELED


def test_exceptions_in_context_action_still_raised(
    threaded_implementer: RobustImplementer, caller: RobustCaller
) -> None:
    """Make sure exceptions that happen remotely are still raised.

    This basically makes sure that get_result() is being called after the context
    exits
    """
    threaded_implementer.raise_error = True

    with (
        pytest.raises(
            RobustRPCException.like(RobustImplementer.OhMyAnError)
        ) as ex_info,
        caller.action_client.send_goal_as_context(
            RobustActionExample.Goal(), timeout=15
        ) as handle,
    ):
        pass

    assert ex_info.value.message.status == GoalStatus.STATUS_ABORTED
    assert handle.status == GoalStatus.STATUS_ABORTED


def test_exception_in_context_cause_cancellation(
    threaded_implementer: RobustImplementer, caller: RobustCaller
) -> None:
    """Test that if an exception is raised in the context, that the underlying action
    is still cancelled

    This ensures that the function uses a `try:  finally:` pattern
    """
    threaded_implementer.action_continue_event = Event()

    with pytest.raises(OSError):
        with caller.action_client.send_goal_as_context(
            RobustActionExample.Goal(), timeout=3
        ) as handle:
            # By not setting the action_continue_event, a timeout is guaranteed
            raise OSError("Oh no, whaaat, some unexpected error?")

        assert handle.status == GoalStatus.STATUS_CANCELED
