from collections.abc import Callable, Generator
from threading import Event
from typing import Any, TypeVar, cast
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from .exceptions import ExceptionCollector

T = TypeVar("T")


def wait_for_future(future: Future, type_: type[T], timeout: float | None = None) -> T:
    result_ready = Event()
    future.add_done_callback(lambda f: result_ready.set())

    future_done: bool = result_ready.wait(timeout=timeout)
    if not future_done:
        raise TimeoutError("Timeout while waiting for the future to finish!")

    # Exceptions may be raised from result()
    result = future.result()

    if isinstance(result, Mock):
        # Allow mocks without question to make testing easier
        return cast(T, result)
    else:
        assert isinstance(
            result, type_
        ), f"Expected result to be of type {type_} but instead got {type(result)}"
        return result


def wait_for_preemptible_future(
    goal_handle: ClientGoalHandle,
    preempt_condition: Callable[[], bool],
    yield_interval: float = 0.01,
) -> bool:
    """
    Run an action asynchronously, allowing for cancellation if a condition is met.
    :param goal_handle: The handle to cancel if necessary
    :param preempt_condition: if true, cancel the goal
    :param yield_interval: How often to yield inbetween waiting for the future.
    :returns: True if the clamps were successfully moved, False otherwise
    """
    on_wood_move_done = Event()
    future = goal_handle.get_result_async()
    future.add_done_callback(lambda f: on_wood_move_done.set())
    while not on_wood_move_done.wait(yield_interval):
        status = goal_handle.status

        if (
            preempt_condition()
            and not future.done()
            and status != GoalStatus.STATUS_CANCELED
            and status != GoalStatus.STATUS_CANCELING
        ):
            goal_handle.cancel_goal()
            goal_handle.get_result()
            return False
    return True


def wait_for_futures(
    futures: list[Future], type_: type[T], timeout: float | None = None
) -> list[T]:
    """
    Block until all futures are complete. If an exception occurs, the first is raised
    """

    results: list[T] = []
    collector = ExceptionCollector()
    for future in futures:
        with collector:
            results.append(wait_for_future(future, type_, timeout))

    collector.maybe_raise()

    return results


def wait_for_send_goal(
    client: ActionClient,
    goal: Any,
    feedback_callback: Callable[[Any], None] | None = None,
) -> tuple[Future, ClientGoalHandle]:
    """A shorthand way to asynchronously call an action and get a get_result future"""
    future = client.send_goal_async(goal, feedback_callback=feedback_callback)
    goal_handle = wait_for_future(future, ClientGoalHandle)
    return goal_handle.get_result_async(), goal_handle


def yield_for_futures(
    futures: list[Future], type_: type[T], yield_interval: float
) -> Generator[None, None, list[T]]:
    """
    Yield until all futures are complete. If an exception occurs, the first is raised
    """

    results: list[T] = []
    collector = ExceptionCollector()
    for future in futures:
        with collector:
            result = yield from yield_for_future(
                future, type_, yield_interval=yield_interval
            )
            results.append(result)

    collector.maybe_raise()

    return results


def yield_for_future(
    future: Future, type_: type[T], yield_interval: float
) -> Generator[None, None, T]:
    """This function will yield `None` until the future is complete, then return results

    This behavior is useful for waiting for actions to finish while allowing
    cancellation in an ActionWorker.

    Usage Example, in an action worker "run" loop. In this example, cancellation will be
    checked every 0.1 seconds, and when the future is finished it will be stored in
    'results'.
    >>> future = client.call_async()
    >>> results = yield from yield_for_future(future, yield_interval=0.1)

    :param future: The future to wait for
    :param type_: The type of the result that will be returned
    :param yield_interval: How often to yield inbetween waiting for the future.
    :yields: None, until all futures are complete
    :return: The resulting value of the future.
    """

    result_ready = Event()
    future.add_done_callback(lambda f: result_ready.set())

    while not result_ready.wait(timeout=yield_interval):
        yield None

    # Exceptions may be raised from result()
    result = future.result()

    assert isinstance(
        result, type_
    ), f"Expected result to be of type {type_} but instead got {type(result)}"
    return result


def run_action_with_timeout(
    client: ActionClient, goal: Any, response_type: type[T], timeout: float = 60.0
) -> T:
    """Run an action and block for the response, raising a TimeoutError if it takes too
    long. This action is intended for test usage, since in production it's typically a
    bad idea to use Timeouts for actions that might still be doing things.
    :param client: The action client
    :param goal: The goal to send
    :param response_type: The expected result type. This can be weird, it's an
        autogenerated object of the name {ActionName}_GetResult_Response
    :param timeout: The number of seconds
    :return: The result of the action
    :raises RuntimeError: If the server shuts down before the action completes
    """

    try:
        handle_future: Future = client.send_goal_async(goal)

        result_future = wait_for_future(
            handle_future, ClientGoalHandle, timeout=timeout
        ).get_result_async()
    except TimeoutError as ex:
        if not client.server_is_ready():
            msg = "The action server shut down before the action completed!"
            raise RuntimeError(msg) from ex
        raise
    return wait_for_future(result_future, response_type, timeout=timeout)


def split_done_futures(futures: list[Future]) -> tuple[list[Future], list[Future]]:
    """Given a list of futures, categorize them into 'done' and unfinished futures
    :param futures: The futures to categorize
    :returns: (done futures, unfinished futures)
    """
    finished: list[Future] = []
    unfinished: list[Future] = []
    for future in futures:
        category = finished if future.done() else unfinished
        category.append(future)
    return finished, unfinished


__all__ = [
    "wait_for_future",
    "run_action_with_timeout",
    "split_done_futures",
    "wait_for_send_goal",
]
