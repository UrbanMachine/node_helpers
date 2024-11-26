from collections.abc import Generator
from threading import Event
from time import sleep
from typing import Any

import pytest
from node_helpers.robust_rpc import RobustRPCException, RobustRPCMixin
from node_helpers.testing import set_up_node
from node_helpers_msgs.action import RobustActionExample
from node_helpers_msgs.srv import RobustServiceExample
from rclpy.action.server import CancelResponse, ServerGoalHandle
from rclpy.node import Node


class RobustCaller(Node, RobustRPCMixin):
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__("caller", *args, **kwargs)

        self.service_client = self.create_robust_client(
            srv_type=RobustServiceExample, srv_name="robust_service"
        )
        self.action_client = self.create_robust_action_client(
            action_type=RobustActionExample, action_name="robust_action"
        )


class RobustImplementer(Node, RobustRPCMixin):
    class OhMyAnError(Exception):
        pass

    ERROR_MSG = "Oh wow an error message? In _this_ code?? I would never. "
    EXPECTED_DATA = "Oh my, cool data that my service/action returned!"
    EXPECTED_CANCELLED_DATA = "Oh my, the action was cancelled"

    ROBUST_RPC_ERROR_NAME = "SomeCoolError"

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__("implementer", *args, **kwargs)
        self.raise_error = False
        self.raise_robust_rpc_error = False
        self.action_cancellable = True

        self.action_reached_event = Event()
        """This event is set when the action is called"""

        self.action_continue_event: Event | None = None
        """If set, the action will wait until this event is set"""

        self.service_calls = 0
        self.action_calls = 0

        self.robust_service = self.create_robust_service(
            srv_type=RobustServiceExample,
            srv_name="robust_service",
            callback=self.service_callback,
        )

        self.robust_action_server = self.create_robust_action_server(
            action_type=RobustActionExample,
            action_name="robust_action",
            execute_callback=self.action_callback,
            cancel_callback=lambda cancel_request: (
                CancelResponse.ACCEPT
                if self.action_cancellable
                else CancelResponse.REJECT
            ),
        )

    def action_callback(self, goal: ServerGoalHandle) -> RobustActionExample.Result:
        self.action_reached_event.set()
        self.action_calls += 1
        self._maybe_exception()

        if self.action_continue_event is not None:
            # Wait for either the action to be cancelled, or for the event to be set
            while not self.action_continue_event.is_set():
                # Check for cancellation requests (requires multithreaded executor)
                if goal.is_cancel_requested:
                    goal.canceled()
                    return RobustActionExample.Result(data=self.EXPECTED_CANCELLED_DATA)
                sleep(0.05)

        goal.succeed()
        return RobustActionExample.Result(data=self.EXPECTED_DATA)

    def service_callback(
        self,
        request: RobustServiceExample.Request,
        response: RobustServiceExample.Response,
    ) -> RobustServiceExample.Response:
        self.service_calls += 1
        self._maybe_exception()
        response.data = self.EXPECTED_DATA
        return response

    def _maybe_exception(self) -> None:
        if self.raise_error:
            raise self.OhMyAnError(self.ERROR_MSG)
        elif self.raise_robust_rpc_error:
            raise RobustRPCException(
                error_name=self.ROBUST_RPC_ERROR_NAME,
                error_description=self.ERROR_MSG,
                message=RobustServiceExample.Request(),
            )


@pytest.fixture()
def caller() -> Generator[RobustCaller, None, None]:
    yield from set_up_node(
        node_class=RobustCaller,
        namespace="robust",
        node_name="caller",
        multi_threaded=False,
    )


@pytest.fixture()
def implementer() -> Generator[RobustImplementer, None, None]:
    yield from set_up_node(
        node_class=RobustImplementer,
        namespace="robust",
        node_name="implementer",
        multi_threaded=False,
    )


@pytest.fixture()
def threaded_implementer() -> Generator[RobustImplementer, None, None]:
    yield from set_up_node(
        node_class=RobustImplementer,
        namespace="robust",
        node_name="implementer",
        multi_threaded=True,
    )
