from threading import RLock
from typing import Any

from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from node_helpers.nodes import HelpfulNode
from node_helpers.robust_rpc.typing import RobustActionMsg

from ._typing import FEEDBACK_TYPE, GOAL_TYPE, RESULT_TYPE
from .base_handler import ActionHandler


class SynchronousActionCalledInParallel(Exception):
    """This exception is raised when an exception that should only ever be called
    synchronously is called in parallel, meaning there's likely a bug somewhere."""


class FailFastActionHandler(ActionHandler[GOAL_TYPE, FEEDBACK_TYPE, RESULT_TYPE]):
    """This action handler will immediately raise an exception if the action is called
    in parallel. This is useful for defining an action that should only ever be called
    synchronously, and you want to prevent situations where the code runs in parallel.
    """

    def __init__(
        self,
        node: HelpfulNode,
        action_name: str,
        action_type: type[RobustActionMsg],
        cancellable: bool = True,
        **action_server_kwargs: dict[str, Any],
    ):
        super().__init__(
            node=node,
            action_name=action_name,
            action_type=action_type,
            callback_group=ReentrantCallbackGroup(),
            cancellable=cancellable,
            metrics_callback=None,
            **action_server_kwargs,
        )

        self.action_lock = RLock()
        """This lock is held while an action is being run"""

    def on_goal(self, goal_handle: ServerGoalHandle) -> RESULT_TYPE:
        acquired = self.action_lock.acquire(blocking=False)
        if not acquired:
            raise SynchronousActionCalledInParallel(
                f"The action '{self.action_name}' was called while another call was in"
                f" progress!"
            )

        try:
            return super().on_goal(goal_handle)
        finally:
            self.action_lock.release()
