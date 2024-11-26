from unittest import mock

import pytest
from node_helpers.robust_rpc import ExecutorNotSetError
from node_helpers.testing import rclpy_context
from node_helpers_msgs.action import RobustActionExample

from node_helpers_test.integration.robust_rpc.conftest import (
    RobustCaller,
    RobustImplementer,
)


def test_waits_for_action(caller: RobustCaller, implementer: RobustImplementer) -> None:
    """Test that actions are waiting for readiness on the first call"""
    with mock.patch.object(
        caller.action_client,
        "wait_for_server",
        wraps=caller.action_client.wait_for_server,
    ) as wait_fn:
        assert wait_fn.call_count == 0

        caller.action_client.send_goal_async(goal=RobustActionExample.Goal())

        assert wait_fn.call_count == 1

        caller.action_client.send_goal_async(goal=RobustActionExample.Goal())
        assert wait_fn.call_count == 1, "Waiting should only occur on the first call!"


def test_fails_without_executor() -> None:
    """Test that attempting to call an action without the executor set fails"""
    with rclpy_context() as context:
        caller = RobustCaller(namespace="something", context=context)

        with pytest.raises(ExecutorNotSetError):
            caller.action_client.send_goal_async(RobustActionExample.Goal())
