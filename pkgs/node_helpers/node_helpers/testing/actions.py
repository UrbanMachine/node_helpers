from collections.abc import Callable
from typing import Any
from unittest.mock import Mock
from uuid import UUID

from action_msgs.msg import GoalStatus
from rclpy import Future

from node_helpers.futures import wait_for_future


class MockClientGoalHandle(Mock):
    """Plays the role of a ClientGoalHandle, but with controllable run times and result
    values
    """

    def __init__(self) -> None:
        super().__init__()
        self._result_future = Future()
        self.cancel_result: Any | None = None

    def get_result(self) -> Any:
        return wait_for_future(self.get_result_async(), type_=object)

    def get_result_async(self) -> Future:
        return self._result_future

    def cancel_goal(self) -> None:
        self.cancel_goal_async()

    @property
    def status(self) -> GoalStatus:
        return GoalStatus.STATUS_SUCCEEDED

    def cancel_goal_async(self) -> Future:
        action_is_already_finished = (
            self._result_future is not None and self._result_future.done()
        )

        if self.cancel_result is None and not action_is_already_finished:
            raise RuntimeError(
                "Goal was cancelled but no cancel result has been set. Either set a "
                "result for the action, or set the cancel result."
            )

        self.set_result(
            self._result_future.result()
            if self.cancel_result is None
            else self.cancel_result
        )

        future = Future()
        # Cancel immediately
        future.set_result(Mock())
        return future

    def set_result(self, result: Any) -> None:
        """Marks the goal handle as complete with the given result"""
        self._result_future.set_result(result)


class MockRobustActionClient(Mock):
    """A RobustActionClient that produces MockClientGoalHandles when goals are sent"""

    def __init__(self) -> None:
        super().__init__()
        self.called_with: list[Any] = []
        self._result: Any | None = None
        self._cancel_result: Any | None = None

    def send_goal(self, goal: Any) -> Any:
        return self.send_goal_async(goal).result()

    def send_goal_async(
        self,
        goal: Any,
        feedback_callback: Callable[[Any], None] | None = None,
        goal_uuid: UUID | None = None,
    ) -> Future:
        self.called_with.append(goal)
        client_goal_handle = MockClientGoalHandle()
        if self._result is not None:
            client_goal_handle.set_result(self._result)
        if self._cancel_result is not None:
            client_goal_handle.cancel_result = self._cancel_result
        future = Future()
        future.set_result(client_goal_handle)
        return future

    def set_result(self, result: Any) -> None:
        """Marks all future MockClientGoalHandles as complete with the given result
        value

        :param result: The result to provide
        """
        self._result = result

    def set_cancel_result(self, result: Any) -> None:
        """Provides this result to all future MockClientGoalHandles that get cancelled.
        MockClientGoalHandles may not be cancelled until they have a cancel result.

        :param result: The result to provide on cancellation
        """
        self._cancel_result = result
