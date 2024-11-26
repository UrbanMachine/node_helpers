import contextlib
from collections.abc import Generator
from threading import RLock
from typing import Any, cast

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from ..futures import wait_for_future
from ._readiness import ValidatesReadinessMixin
from ._wrappers import patch_future_to_raise_exception
from .errors import ActionCancellationRejected
from .typing import RobustActionMsg


class RobustActionClient(ActionClient, ValidatesReadinessMixin):
    """Wraps an ActionClient so that any exceptions raised by the remote server are
    raised in the client, when calling send_goal_async().result() or send_goal()
    """

    @contextlib.contextmanager
    def send_goal_as_context(
        self,
        goal: RobustActionMsg,
        timeout: float | None = None,
        cancel: bool = True,
        block_for_result: bool = True,
        **kwargs: Any,
    ) -> Generator[ClientGoalHandle, None, None]:
        """Create a context manager that will send a goal, then after the context cancel
         and wait for a result.

         Note: There are no guarantees that by the time the context is entered, that the
         action is running yet.

        :param goal: The goal to send
        :param timeout: The timeout for the goal to expected, and for the goal to be
            cancelled. Helpful for when writing tests.
        :param cancel: If True, the action will be cancelled if its not already finished
            when the context ends. If the cancellation is rejected, an exception will
            be raised.
        :param block_for_result: If True, the context manager will block for a result
            when exiting
        :param kwargs: Any other send_goal_async parameters
        :yields: The ClientGoalHandle of the created action

        :raises ActionCancellationRejected: If cancel=True but the server rejects the
            cancellation request.
        """
        goal_handle = wait_for_future(
            self.send_goal_async(goal=goal, **kwargs), ClientGoalHandle, timeout=timeout
        )

        try:
            yield goal_handle
        finally:
            # Cancel the goal if it hasn't finished yet
            if cancel and goal_handle.status in [
                GoalStatus.STATUS_UNKNOWN,
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
            ]:
                cancel_response = wait_for_future(
                    goal_handle.cancel_goal_async(),
                    type_=CancelGoal.Response,
                    timeout=timeout,
                )

                action_aborted = (
                    cancel_response.return_code
                    == CancelGoal.Response.ERROR_GOAL_TERMINATED
                )

                # If the goal was not cancelled for reasons other than a remote error,
                # raise an exception.
                if len(cancel_response.goals_canceling) == 0 and not action_aborted:
                    raise ActionCancellationRejected(
                        f"Action {self._action_name} is required to be cancellable, but"
                        f" the cancellation was rejected! "
                        f"Cancellation Code: {cancel_response.return_code}"
                    )

            if block_for_result:
                # Wait for the goal to finish cancelling
                wait_for_future(goal_handle.get_result_async(), object, timeout=timeout)

    def send_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        self._validate_rpc_server_is_ready(
            wait_fn=lambda: cast(bool, self.wait_for_server(10)),
            rpc_name=self._action_name,
        )

        return super().send_goal_async(*args, **kwargs)

    def _get_result_async(self, goal_handle: ClientGoalHandle) -> Future:
        """Patch the future so that when result() is called it raises remote errors"""

        future = super()._get_result_async(goal_handle)
        patch_future_to_raise_exception(future=future, parse_as_action=True)
        return future


class PatchRclpyIssue1123(RobustActionClient):
    """Remove this patch after the issue https://github.com/ros2/rclpy/issues/1123 is
    resolved.

    Tracking PR: https://github.com/ros2/rclpy/pull/1125

    The short summary here: This hacky patch fixes an issue where state within the
    ActionClient.*_async() functions had a race condition with what ActionClient.execute
    does, resulting in Actions that would never complete.

    Please remove this hack once this issue is resolved upstream.
    """

    _lock: RLock = None  # type: ignore

    @property
    def _cpp_client_handle_lock(self) -> RLock:
        if self._lock is None:
            self._lock = RLock()
        return self._lock

    async def execute(self, *args: Any, **kwargs: Any) -> None:
        # This is ugly- holding on to a lock in an async environment feels gross
        with self._cpp_client_handle_lock:
            return await super().execute(*args, **kwargs)  # type: ignore

    def send_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super().send_goal_async(*args, **kwargs)

    def _cancel_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._cancel_goal_async(*args, **kwargs)

    def _get_result_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._get_result_async(*args, **kwargs)


# Replace the robust action client with its patched variant
RobustActionClient = PatchRclpyIssue1123  # type: ignore
