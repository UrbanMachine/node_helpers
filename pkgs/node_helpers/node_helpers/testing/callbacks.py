from collections.abc import Iterable
from threading import Event, RLock
from time import sleep
from typing import Any, Generic, TypeVar, cast

from rclpy.action.server import ServerGoalHandle

from node_helpers.pubsub import PublishEvent

ReturnType = TypeVar("ReturnType")


class ConfigurableServiceCallback(Generic[ReturnType]):
    """This class is for creating a service that returns preconfigured values each
    iteration."""

    def __init__(self, return_values: Iterable[ReturnType]):
        self._return_value_iterator = iter(return_values)
        self._iterator_lock = RLock()
        self.call_count: int = 0

    def set_return_values(self, return_values: Iterable[ReturnType]) -> None:
        """This method is useful for tests that want to override the default retval"""
        with self._iterator_lock:
            self._return_value_iterator = iter(return_values)

    def __call__(self, *args: Any, **kwargs: Any) -> ReturnType:
        with self._iterator_lock:
            self.call_count += 1
            return next(self._return_value_iterator)


class ActionServerCallback(Generic[ReturnType]):
    def __init__(
        self, return_values: Iterable[ReturnType], feedback_values: Iterable[Any] = ()
    ) -> None:
        """A helper for creating action callbacks that block, check cancellation, return
        and otherwise allow control from the outside to emulate some behavior or
        another.

        Instantiation example:
        >>> action_callback = ActionServerCallback(
        >>>         (SomeType.Response(1),  SomeType.Response(2))
        >>> )
        >>> node.create_robust_service(
        >>>    SomeType,
        >>>    "some_name"
        >>>    callback_group=MutuallyExclusiveCallbackGroup(),
        >>>    callback=action_callback,
        >>>    cancel_callback=lambda *args: CancelResponse.ACCEPT
        >>> )

        # Usage example, where testing if an action calls cancel on a child action:
        >>> some_client.send_goal_async()
        >>> assert action_callback.action_has_started.wait(30)
        >>> some_client.cancel_goals_async()
        >>> assert action_callback.on_cancel_requested.wait(30)
        >>> # Do some other stuff, then allow the cancellation to continue
        >>> some_client.allow_cancel.set()

        :param return_values: An iterable object of the action responses
        :param feedback_values: An iterable object of the action feedback
        """
        self.on_action_started = PublishEvent()
        self.on_action_exiting = PublishEvent()
        self.on_cancel_requested = PublishEvent()

        # Users of this action can set these events when they are ready for the action
        # to finish.
        self.allow_succeed = Event()
        self.allow_abort = Event()
        self.allow_cancel = Event()

        # Feedback related controls (will publish feedback once per event set)
        self.allow_publish_feedback = Event()

        # This can be used to introspect the ServerGoalHandle
        self.ongoing_goal_handle: ServerGoalHandle | None = None

        self._return_value_iterator = ConfigurableServiceCallback(return_values)
        self._feedback_iterator = ConfigurableServiceCallback(feedback_values)

    @property
    def call_count(self) -> int:
        return self._return_value_iterator.call_count

    def set_return_values(self, return_values: Iterable[ReturnType]) -> None:
        """Allows overriding the default action 'return' value list"""
        self._return_value_iterator.set_return_values(return_values)

    def set_feedback_values(self, feedback_values: Iterable[Any]) -> None:
        """Allows overriding the default action 'feedback' value list"""
        self._feedback_iterator.set_return_values(feedback_values)

    @property
    def ongoing_goal(self) -> Any:
        return cast(ServerGoalHandle, self.ongoing_goal_handle).request

    def __call__(self, goal: ServerGoalHandle) -> ReturnType:
        # Track the current goal handle, for test writing
        if self.ongoing_goal_handle is not None:
            raise RuntimeError(
                "ActionServerCallback isn't designed for multithreaded use!"
            )
        self.ongoing_goal_handle = goal

        # Now that 'ongoing_goal_handle' is tracked, mark the action as 'started'
        self.on_action_started.set()

        # Wait for the action to be allowed to either finish or be cancelled
        while True:
            if self.allow_publish_feedback.is_set():
                feedback = self._feedback_iterator()
                goal.publish_feedback(feedback)

            if self.allow_succeed.is_set():
                goal.succeed()
                break

            if self.allow_abort.is_set():
                goal.abort()
                break

            if goal.is_cancel_requested:
                self.on_cancel_requested.set()

                if self.allow_cancel.is_set():
                    goal.canceled()
                    break
            sleep(0.1)

        self.on_action_exiting.set()
        self.ongoing_goal_handle = None

        return self._return_value_iterator()
