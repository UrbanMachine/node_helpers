from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Generic, Literal
from uuid import UUID

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rclpy.action.server import CancelResponse, ServerGoalHandle
from rclpy.callback_groups import CallbackGroup

from node_helpers.nodes import HelpfulNode
from node_helpers.robust_rpc import RobustRPCException
from node_helpers.robust_rpc.typing import RobustActionMsg
from node_helpers.timing import Timer

from ._typing import FEEDBACK_TYPE, GOAL_TYPE, RESULT_TYPE
from .worker import ActionWorker


@dataclass
class ActionCallMetric:
    """A metric to be reported on an action, and passed to the metrics_callback"""

    action_name: str
    node_namespace: str
    goal_id: UUID

    elapsed: float | None = None
    error_name: str | None = None
    error_description: str | None = None
    result: Literal["success", "error", "canceled", "in_progress"] | None = None


class ActionHandler(ABC, Generic[GOAL_TYPE, FEEDBACK_TYPE, RESULT_TYPE]):
    """Creates and runs an ActionWorker instance when an action request is received"""

    def __init__(
        self,
        node: HelpfulNode,
        action_name: str,
        action_type: type[RobustActionMsg],
        callback_group: CallbackGroup,
        cancellable: bool = True,
        metrics_callback: Callable[[ActionCallMetric], None] | None = None,
        **action_server_kwargs: Any,
    ):
        """
        :param node: The node to create the action server on
        :param action_name: The name of the action
        :param action_type: The ROS message type of the action
        :param callback_group: The callback group to use for the action server
        :param cancellable: Whether the action can be canceled
        :param metrics_callback: A callback to call when metrics are reported. This is
            useful for testing or for hooking up to a metrics system that records all
            action calls to a database.

            This will be called 2 times for each action call:
            - When the action is first called, with a result of "in_progress"
            - When the action is completed with "success", "error", or "canceled"

        :param action_server_kwargs: Additional keyword arguments to pass to the action
        """

        self._request_timeout: float | None = node.declare_and_get_parameter(
            f"{action_name}_action_timeout",
            type_=float,
            description=f"The timeout, in seconds, to use for the {action_name} action",
            default_value=0.0,
        )

        # If the default is not set, don't time out
        self._request_timeout = self._request_timeout or None

        self.node = node
        self.action_name = action_name
        self._callback_group = callback_group

        self._metrics_callback = metrics_callback or (lambda _: None)

        def cancel_callback(_request: CancelGoal.Request) -> CancelResponse:
            return CancelResponse.ACCEPT if cancellable else CancelResponse.REJECT

        self._server = node.create_robust_action_server(
            action_type=action_type,
            action_name=action_name,
            execute_callback=self._on_goal_with_report,
            callback_group=self._callback_group,
            cancel_callback=cancel_callback,
            **action_server_kwargs,
        )

    @abstractmethod
    def create_worker(
        self, goal_handle: ServerGoalHandle
    ) -> ActionWorker[GOAL_TYPE, FEEDBACK_TYPE, RESULT_TYPE]:
        """Called when a worker needs to be made for a new goal"""

    def on_goal(self, goal_handle: ServerGoalHandle) -> RESULT_TYPE:
        """Called when a goal request is received. May be overridden by subclasses."""
        worker = self.create_worker(goal_handle)
        return worker.execute_callback(self._request_timeout)

    def _on_goal_with_report(self, goal_handle: ServerGoalHandle) -> RESULT_TYPE:
        """Wraps _on_goal and publishes a metric reporting information on the action"""
        metric = ActionCallMetric(
            action_name=self.action_name,
            node_namespace=self.node.get_namespace(),
            goal_id=UUID(bytes=bytes(goal_handle.goal_id.uuid)),
            result="in_progress",
        )

        # Always publish an 'in_progress' report immediately
        self._metrics_callback(metric)

        timer = Timer()
        try:
            with timer:
                result = self.on_goal(goal_handle)

                # If there were no errors, that's considered a success!
                metric.result = "success"
                return result
        except Exception as ex:
            # Record information on the error, including a description from the error if
            # it's available
            if isinstance(ex, RobustRPCException):
                metric.error_name = ex.error_name
                metric.error_description = ex.error_description
            else:
                metric.error_name = type(ex).__name__
                metric.error_description = str(ex)
            metric.result = "error"
            raise
        finally:
            if goal_handle.status == GoalStatus.STATUS_CANCELED:
                metric.result = "canceled"

            # Regardless of the result status, always record the final time
            metric.elapsed = timer.elapsed
            self._metrics_callback(metric)
