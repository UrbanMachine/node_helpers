from collections.abc import Callable

from rclpy.callback_groups import CallbackGroup
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.timer import Timer

from node_helpers.timing import WarningTimer


class TimerWithWarningsMixin:
    """This mixin adds a method for creating a timer that will log a warning when the
    timer is falling behind due to the callback taking too long.
    """

    def create_timer_with_warnings(
        self: Node,
        timer_period_sec: float,
        callback: Callable[[], None],
        name: str,
        callback_group: CallbackGroup = None,
        clock: Clock = None,
    ) -> Timer:
        warning_timer = WarningTimer(name, 1 / timer_period_sec)

        def wrapped() -> None:
            with warning_timer:
                callback()

        return self.create_timer(
            timer_period_sec=timer_period_sec,
            callback=wrapped,
            callback_group=callback_group,
            clock=clock,
        )
