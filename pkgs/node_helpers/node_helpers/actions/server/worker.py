import logging
from abc import ABC, abstractmethod
from collections.abc import Generator
from typing import Generic, cast

from rclpy.action.server import ServerGoalHandle

from node_helpers.timing import Timeout

from ._typing import FEEDBACK_TYPE, GOAL_TYPE, RESULT_TYPE


class ActionTimeoutError(Exception):
    """The action took longer than the timeout to run"""


class NoResultSetError(Exception):
    """The action implementation completed successfully, but no result value was set"""


class ActionWorker(ABC, Generic[GOAL_TYPE, FEEDBACK_TYPE, RESULT_TYPE]):
    def __init__(self, goal_handle: ServerGoalHandle):
        self.goal_handle = goal_handle
        """A handle used to communicate with the caller. Implementations usually do not
        need to use this object directly. See ``run`` for details.
        """

        self.goal = cast(GOAL_TYPE, goal_handle.request)
        """The goal data sent by the caller, containing information on the request"""

        self.done = False
        """True if the worker is no longer running, including as a result of an error
        or cancellation
        """

        self.result: RESULT_TYPE | None = None
        """The result value to provide to the caller. This must be set by ``run`` if
        the work completed successfully.
        """

    @abstractmethod
    def run(self) -> Generator[FEEDBACK_TYPE | None, None, None]:
        """Triggered when an action begins execution. This function is expected to
        regularly yield (feedback objects or None) and set the ``result`` attribute
        before returning.

        This method will be automatically interrupted after a yield if a timeout is
        reached or if the action is canceled. This means that, for best operation,
        implementations of this method should yield regularly.
        """

    @abstractmethod
    def on_cancel(self) -> RESULT_TYPE:
        """Triggered when the client cancels the request

        :return: The result to provide to the client
        """

    def on_exception(self, ex: Exception) -> None:
        """Triggered when an exception happens in the ``run`` method"""

    def execute_callback(self, timeout: float | None) -> RESULT_TYPE:
        """Runs the action, blocking until it's finished

        :param timeout: The maximum allowed time for the action to take, or None for
            no limit
        :return: The result to provide to the client
        :raises ActionTimeoutError: If the action is taking longer than the provided
            timeout
        :raises NoResultSetError: If the ``run`` method succeeds, but does not set the
            result attribute
        :raises RuntimeError: If something strange happens
        """

        try:
            run_generator = self.run()

            timeout_obj: bool | Timeout = (
                Timeout(timeout) if timeout is not None else True
            )

            while timeout_obj and not self.goal_handle.is_cancel_requested:
                try:
                    feedback = next(run_generator)
                    if feedback is not None:
                        self.goal_handle.publish_feedback(feedback)
                except StopIteration as e:
                    if self.result is None:
                        raise NoResultSetError(
                            "Action failed to produce a result"
                        ) from e
                    self.goal_handle.succeed()
                    return self.result
                except Exception as ex:
                    self.on_exception(ex)
                    raise

            if self.goal_handle.is_cancel_requested:
                logging.debug(f"Canceling action. Goal={self.goal}")

                # Forces immediate garbage collection on the generator, ensuring that
                # context managers and `finally` blocks are run before cancellation.
                del run_generator

                result = self.on_cancel()
                self.goal_handle.canceled()
                return result
            elif not timeout_obj:
                raise ActionTimeoutError("Timeout while running the action")

            raise RuntimeError(
                "Action stopped running before a cancel request or timeout! This "
                "should never happen!"
            )
        finally:
            self.done = True
