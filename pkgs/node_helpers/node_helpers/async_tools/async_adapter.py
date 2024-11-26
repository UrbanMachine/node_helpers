import asyncio
import functools
from collections.abc import Callable, Coroutine
from concurrent.futures import Future as ThreadingFuture
from threading import Event as ThreadingEvent
from typing import Any, TypeVar

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from node_helpers.nodes import HelpfulNode

_CORO_RETURN = TypeVar("_CORO_RETURN")


class AsyncAdapter:
    """Provides a bridge between asyncio code and a traditional ROS node with a
    multithreaded executor. This takes care of creating an event loop that runs in the
    background, separate from ROS's event loop.

    Rclpy actually does have some native support for using coroutines as callbacks, but
    that support is not well-documented and has some limitations. Namely, the coroutines
    are run directly as generators instead of through an asyncio event loop. This
    (I think) means that asyncio networking won't work, which we need.

    In the future, we could consider running the Rclpy event loop inside an asyncio
    event loop.
    """

    def __init__(self, node: HelpfulNode):
        self.event_loop = asyncio.get_event_loop()

        node.create_single_shot_timer(
            0.0,
            self._run_event_loop,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        node.on_destroy(self._stop_event_loop)

    def adapt(
        self, callback: Callable[..., Coroutine[Any, Any, _CORO_RETURN]]
    ) -> Callable[..., _CORO_RETURN]:
        """Adapts an async function to be run in the event loop of the node when called.
        The resulting function will block until the coroutine has completed. This is
        intended for use with topic and service subscriber callbacks.

        :param callback: The async function to wrap
        :return: The return value of the coroutine once it's completed
        """

        @functools.wraps(callback)
        def wrapper(*args: Any, **kwargs: Any) -> _CORO_RETURN:
            future: ThreadingFuture[_CORO_RETURN] = ThreadingFuture()

            def run_task() -> None:
                new_task = self.event_loop.create_task(callback(*args, **kwargs))

                def on_done(task: asyncio.Task[_CORO_RETURN]) -> None:
                    if task.exception():
                        future.set_exception(task.exception())
                    else:
                        future.set_result(task.result())

                new_task.add_done_callback(on_done)

            self.event_loop.call_soon_threadsafe(run_task)
            return future.result()

        return wrapper

    def _run_event_loop(self) -> None:
        try:
            self.event_loop.run_forever()
        finally:
            self.event_loop.close()

    def _stop_event_loop(self) -> None:
        stopped = ThreadingEvent()

        def stop() -> None:
            self.event_loop.stop()
            stopped.set()

        self.event_loop.call_soon_threadsafe(stop)
        if not stopped.wait(5.0):
            raise RuntimeError("Timeout while stopping asyncio event loop")
