from collections.abc import Callable

from rclpy.callback_groups import CallbackGroup
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.timer import Timer


class _TimerCallback:
    def __init__(self, callback: Callable[[], None]):
        self._timer: Timer = None  # Is assigned right after creation
        self._callback = callback

    def __call__(self) -> None:
        # Cancel and destroy the timer, then remove references to it
        self._timer.cancel()
        self._timer.destroy()
        self._timer = None

        return self._callback()

    def assign_timer(self, timer: Timer) -> None:
        """Called once the timer has been created with this callback"""
        self._timer = timer


class SingleShotMixin:
    """This mixin adds a method for creating a timer that will execute only once.

    The benefits of using this mixin is:
    1) It is tested and verified to not leave any references.
    2) Less boilerplate
    """

    def create_single_shot_timer(
        self: Node,
        timer_period_sec: float,
        callback: Callable[[], None],
        callback_group: CallbackGroup = None,
        clock: Clock = None,
    ) -> Timer:
        timer_callback = _TimerCallback(callback)
        timer = self.create_timer(
            timer_period_sec=timer_period_sec,
            callback=timer_callback,
            callback_group=callback_group,
            clock=clock,
        )
        timer_callback.assign_timer(timer)

        return timer
