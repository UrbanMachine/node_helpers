from collections.abc import Callable
from typing import Any, cast
from unittest import mock

import pytest
from action_msgs.msg import GoalStatus
from node_helpers.futures import run_action_with_timeout, wait_for_future
from node_helpers.robust_rpc import mixin
from node_helpers.robust_rpc.errors import InvalidRobustMessage, RobustRPCException
from node_helpers.robust_rpc.schema import validate_robust_message
from node_helpers.robust_rpc.typing import (
    RequestType,
    ResponseType,
    RobustActionMsg,
    RobustServiceMsg,
)
from node_helpers_msgs.action import RobustActionExample
from node_helpers_msgs.action._robust_action_example import (
    RobustActionExample_GetResult_Response,
)
from node_helpers_msgs.srv import RobustServiceExample
from rclpy.action.server import ServerGoalHandle
from sensor_msgs.srv import SetCameraInfo
from visualization_msgs.srv import GetInteractiveMarkers

from .conftest import RobustCaller, RobustImplementer


def _validate_successful_message(
    msg: RobustServiceExample.Response | RobustActionExample.Result,
) -> None:
    assert msg.data == RobustImplementer.EXPECTED_DATA
    assert msg.error_name == ""
    assert msg.error_description == ""


def _validate_exception(
    ex_info: RobustRPCException,
    expected_error_type: type[RobustRPCException],
    expected_message_type: type[ResponseType],
) -> None:
    """Validate that the exception passed along the expected values"""
    # Validate error attributes
    assert ex_info.__class__.__name__ == ex_info.error_name
    assert type(ex_info) is RobustRPCException.like(
        ex_info.error_name
    ), "These must be exactly the same error object"
    assert isinstance(ex_info, RobustRPCException)
    assert isinstance(ex_info, expected_error_type)

    assert ex_info.error_name == RobustImplementer.OhMyAnError.__name__
    assert ex_info.error_description == RobustImplementer.ERROR_MSG

    # Validate the error message also includes the underlying exception information
    assert RobustImplementer.ERROR_MSG in str(
        ex_info
    ), "The error message should be passed in the error message!"
    assert RobustImplementer.OhMyAnError.__name__ in str(
        ex_info
    ), "The original error name should be passed in the error message"


def test_robust_service_exception(
    caller: RobustCaller, implementer: RobustImplementer
) -> None:
    """Test a service with a callback exception propagates to the client"""
    implementer.raise_error = True
    future = caller.service_client.call_async(request=RobustServiceExample.Request())

    expected_error_type = RobustRPCException.like(RobustImplementer.OhMyAnError)
    with pytest.raises(expected_error_type) as ex_info:
        wait_for_future(future, RobustServiceExample.Response, timeout=30)
    assert implementer.service_calls == 1 and implementer.action_calls == 0
    _validate_exception(
        ex_info.value, expected_error_type, RobustServiceExample.Response
    )


def test_robust_service_success(
    caller: RobustCaller, implementer: RobustImplementer
) -> None:
    """Test a service without a callback exception works like normal"""

    implementer.raise_error = False
    future = caller.service_client.call_async(request=RobustServiceExample.Request())
    response = wait_for_future(future, RobustServiceExample.Response, timeout=30)

    assert implementer.service_calls == 1 and implementer.action_calls == 0
    _validate_successful_message(response)


def test_robust_action_exception(
    caller: RobustCaller, implementer: RobustImplementer
) -> None:
    """Test an action with a callback exception propagates to the client"""

    implementer.raise_error = True

    expected_error_type = RobustRPCException.like(RobustImplementer.OhMyAnError)
    with pytest.raises(expected_error_type) as ex_info:
        run_action_with_timeout(
            caller.action_client,
            RobustActionExample.Goal(),
            RobustActionExample_GetResult_Response,
        )
    exception_object = ex_info.value

    assert implementer.service_calls == 0 and implementer.action_calls == 1
    assert (
        exception_object.message.status == GoalStatus.STATUS_ABORTED
    ), "Actions that fail in errors should have the goal status set to aborted!"
    _validate_exception(
        exception_object, expected_error_type, RobustActionExample_GetResult_Response
    )


def test_robust_action_success(
    caller: RobustCaller, implementer: RobustImplementer
) -> None:
    """Test an action without a callback exception works like normal"""

    implementer.raise_error = False
    response = run_action_with_timeout(
        caller.action_client,
        RobustActionExample.Goal(),
        RobustActionExample_GetResult_Response,
    )

    assert response.status is GoalStatus.STATUS_SUCCEEDED
    assert implementer.service_calls == 0 and implementer.action_calls == 1
    _validate_successful_message(response.result)


@pytest.mark.parametrize(
    ("message", "expect_error", "is_action"),
    (
        # Test a valid service
        (RobustServiceExample, False, False),
        # Test invalid service(s)
        (GetInteractiveMarkers, True, False),
        (SetCameraInfo, True, False),
        # Test a valid action
        (RobustActionExample, False, True),
    ),
)
def test_messages_are_checked_for_validity(
    caller: RobustCaller,
    message: type[RobustActionMsg] | type[RobustServiceMsg],
    expect_error: bool,
    is_action: bool,
) -> None:
    """Test that creating a service or action checks the message for fields required"""
    test_fns: list[Callable[[], Any]] = []

    if is_action:

        def fake_action_callback(goal_handle: ServerGoalHandle) -> ResponseType:
            return cast(ResponseType, None)

        action_msg = cast(type[RobustActionMsg], message)
        test_fns.append(lambda: caller.create_robust_action_client(action_msg, "name"))
        test_fns.append(
            lambda: caller.create_robust_action_server(
                action_msg, "name", fake_action_callback
            )
        )
    else:

        def fake_service_callback(a: RequestType, b: ResponseType) -> ResponseType:
            return cast(ResponseType, None)

        service_msg = cast(type[RobustServiceMsg], message)
        test_fns.append(lambda: caller.create_robust_client(service_msg, "name"))
        test_fns.append(
            lambda: caller.create_robust_service(
                service_msg, "name", fake_service_callback
            )
        )

    with mock.patch(
        f"{mixin.__name__}.validate_robust_message", wraps=validate_robust_message
    ) as validate:
        for fn in test_fns:
            if expect_error:
                with pytest.raises(InvalidRobustMessage):
                    fn()
            else:
                fn()
        assert validate.call_count == len(
            test_fns
        ), "The validate call should be called for all functions, always!"


def test_robust_rpc_error_passthrough(
    caller: RobustCaller, implementer: RobustImplementer
) -> None:
    """Tests that, if an action server raises a RobustRPCError, the error name and
    description are passed through to the caller unchanged. In practice, this tends to
    come up when one action calls another action which raises an exception.
    """

    implementer.raise_robust_rpc_error = True
    expected_error_type = RobustRPCException.like(implementer.ROBUST_RPC_ERROR_NAME)
    with pytest.raises(expected_error_type) as ex_info:
        run_action_with_timeout(
            caller.action_client,
            RobustActionExample.Goal(),
            RobustActionExample_GetResult_Response,
        )
    exception_object = ex_info.value

    assert exception_object.error_name == implementer.ROBUST_RPC_ERROR_NAME
    assert exception_object.error_description == implementer.ERROR_MSG
