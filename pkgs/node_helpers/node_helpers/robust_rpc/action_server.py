import logging
from typing import Any

from rclpy.action import ActionServer


class _PatchedRclpyIssue1236(ActionServer):
    """
    TODO: Remove this class once we upgrade from Humble to Jazzy, assuming it's fixed
          Tracking issue: https://github.com/ros2/rclpy/issues/1236
    """

    async def _execute_goal(self, execute_callback: Any, goal_handle: Any) -> Any:
        try:
            return await super()._execute_goal(execute_callback, goal_handle)
        except KeyError:
            logging.error(
                "Caught KeyError in action server, ignoring it and returning."
                f" Happened in function=_execute_goal, "
                f"with args {execute_callback=} {goal_handle=}"
            )

    async def _execute_get_result_request(self, request_header_and_message: Any) -> Any:
        try:
            return await super()._execute_get_result_request(request_header_and_message)
        except KeyError:
            logging.error(
                "Caught KeyError in action server, ignoring it and returning."
                f" Happened in function=_execute_get_result_request, "
                f"with args {request_header_and_message=}"
            )


RobustActionServer = _PatchedRclpyIssue1236
