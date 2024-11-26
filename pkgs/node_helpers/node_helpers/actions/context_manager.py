from collections.abc import Callable
from contextlib import AbstractContextManager
from threading import Event
from types import TracebackType
from typing import Any, cast

from rclpy import Future
from rclpy.action.client import ClientGoalHandle

from node_helpers.futures import wait_for_future
from node_helpers.robust_rpc import RobustActionClient


class ActionContextManager:
    """A generic wrapper that turns specifically designed, long-running actions into
    context managers.

    This is intended for use with actions that start some operation, produce a feedback
    message indicating that the operation has started, and then run indefinitely until
    they are cancelled.

    By default, ActionContextManager synchronously waits for feedback on enter and waits
    for cancellation on exit.

    >>> with ActionContextManager(client, goal) as mgr:
    >>>     # Server provided feedback by this point
    >>>     # Do some work

    In async mode, the context manager does not wait for feedback when entered and
    asynchronously cancels the goal when exited. At any time while the context manager
    is active, the user can choose to wait for feedback to be received. After the
    context manager has exited, the user can wait for cancellation to finish.

    >>> with ActionContextManager(client, goal, async_=True) as mgr:
    >>>     # Do some work
    >>>     mgr.wait_for_feedback()
    >>>     # Do some more work
    >>>
    >>> mgr.wait_for_cancellation()
    """

    def __init__(
        self,
        client: RobustActionClient,
        goal: Any,
        timeout: float | None = None,
        async_: bool = False,
        on_feedback: Callable[[Any], None] | None = None,
    ):
        """
        :param client: A client for the action to call
            Make sure your client has feedback_sub_qos_profile set to
            qos_profile_services_default, or else it might not robustly receive feedback
        :param goal: The goal to send through the client
        :param async_: Controls asynchronous mode
        :param timeout: A timeout to use when calling and cancelling the action. If
            None, no timeout is used.
        :param on_feedback: An optional callback to call when feedback is received.
            Streams the {MESSAGE}_Feedback object as they come in.
        """
        self._client = client
        self._goal = goal
        self._goal_context: AbstractContextManager[ClientGoalHandle] | None = None
        self._timeout = timeout
        self._got_feedback = Event()
        self._result_future: Future | None = None
        self._async = async_
        self._on_feedback = on_feedback

    def __enter__(self) -> "ActionContextManager":
        # Track when the first feedback is received by triggering 'got_feedback'
        def _feedback_wrapper(feedback: Any) -> None:
            self._got_feedback.set()
            if self._on_feedback is not None:
                # Pass on the feedback to the user's callback, if one was provided
                self._on_feedback(feedback.feedback)

        self._goal_context = self._client.send_goal_as_context(
            self._goal,
            feedback_callback=_feedback_wrapper,
            timeout=self._timeout,
            block_for_result=False,
        )
        client_handle = self._goal_context.__enter__()
        self._result_future = client_handle.get_result_async()

        if not self._async:
            self.wait_for_feedback()

        return self

    def activate(self) -> "ActionContextManager":
        return self.__enter__()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        if self._goal_context is not None:
            self._goal_context.__exit__(exc_type, exc_val, exc_tb)
            self._goal_context = None
        if not self._async:
            self.wait_for_cancellation()

        self.check_for_exceptions()

    def deactivate(self) -> None:
        if self.activated:
            return self.__exit__(None, None, None)

    def check_for_exceptions(self) -> None:
        """Check for and raise any remote exceptions"""
        if self._result_future is None:
            raise RuntimeError("check_for_exceptions called before entering context")

        if self._result_future.done():
            self._result_future.result()

    def wait_for_feedback(self) -> None:
        if self._result_future is None:
            raise RuntimeError("wait_for_feedback called before entering context")

        while not self._got_feedback.wait(0.01):
            # Check if the action finished on its own. This is usually because of an
            # exception, but could also happen due to a bug on the server side.
            if self._result_future.done():
                self._result_future.result()  # Potentially raise an exception
                raise RuntimeError("Action finished on its own without being cancelled")

    def wait_for_cancellation(self) -> None:
        wait_for_future(self._result_future, object, timeout=self._timeout)

    @property
    def done(self) -> bool:
        if self._result_future is None:
            raise RuntimeError("Done called before entering context")
        return cast(bool, self._result_future.done())

    @property
    def activated(self) -> bool:
        return self._goal_context is not None
