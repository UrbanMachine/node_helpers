from collections.abc import Generator
from threading import Event

from node_helpers.actions.server import ActionWorker
from node_helpers.timing import TestingTimeout as Timeout
from node_helpers_msgs.action import RobustActionExample
from rclpy.action.server import ServerGoalHandle


class CoolError(Exception):
    pass


class ActionWorkerForTesting(
    ActionWorker[
        RobustActionExample.Goal,
        RobustActionExample.Feedback,
        RobustActionExample.Result,
    ]
):
    """An ActionWorker that can simulate a variety of behaviors based on how it's
    configured
    """

    def __init__(
        self,
        goal_handle: ServerGoalHandle,
        done_event: Event | None = None,
        raise_exception: bool = False,
        on_exception_called: Event | None = None,
    ):
        """
        :param goal_handle: A handle to the current goal
        :param done_event: If provided, the action will wait on this event before
            finishing, yielding routinely
        :param raise_exception: If provided, the action will raise an exception instead
            of completing
        :param on_exception_called: Will be set if an exception occurs. This must be
            set if raise_exception is True.
        """
        super().__init__(goal_handle)
        self.done_event = done_event
        self.raise_exception = raise_exception
        self.on_exception_called = on_exception_called
        if self.on_exception_called is not None:
            self.on_exception_called.clear()

    def run(self) -> Generator[RobustActionExample.Feedback | None, None, None]:
        yield None
        if self.raise_exception:
            raise CoolError("I might be an error, but dang I'm cool")

        # Wait for a done event if it's set
        if self.done_event is not None:
            timeout = Timeout(10)
            while not self.done_event.wait(timeout=0.1) and timeout:
                yield RobustActionExample.Feedback()

        self.result = RobustActionExample.Result(data="success")

    def on_cancel(self) -> RobustActionExample.Result:
        return RobustActionExample.Result(data="canceled")

    def on_exception(self, ex: Exception) -> None:
        if self.on_exception_called is None:
            raise RuntimeError("An exception occurred but the event object is not set")
        self.on_exception_called.set()
