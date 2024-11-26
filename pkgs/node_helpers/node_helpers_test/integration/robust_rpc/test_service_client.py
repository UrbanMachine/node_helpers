from unittest import mock

import pytest
from node_helpers.robust_rpc import ExecutorNotSetError
from node_helpers.testing import rclpy_context
from node_helpers_msgs.srv import RobustServiceExample

from .conftest import RobustCaller, RobustImplementer


def test_waits_for_service(
    caller: RobustCaller, implementer: RobustImplementer
) -> None:
    """Test that actions are waiting for readiness on the first call"""
    with mock.patch.object(
        caller.service_client,
        "wait_for_service",
        wraps=caller.service_client.wait_for_service,
    ) as wait_fn:
        assert wait_fn.call_count == 0

        caller.service_client.call(RobustServiceExample.Request())

        assert wait_fn.call_count == 1

        caller.service_client.call(RobustServiceExample.Request())

        assert wait_fn.call_count == 1, "Waiting should only occur on the first call!"


def test_fails_without_executor() -> None:
    """Test that attempting to call a service without the executor set fails"""
    with rclpy_context() as context:
        caller = RobustCaller(namespace="whatever", context=context)

        with pytest.raises(ExecutorNotSetError):
            caller.service_client.call_async(RobustServiceExample.Request())
