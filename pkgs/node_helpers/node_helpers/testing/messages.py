import math
import queue
from collections.abc import Callable, Iterable
from time import sleep
from typing import Any, TypeAlias, TypeVar, cast

from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .threads import ContextThread


class ConstantPublisher(ContextThread):
    """Constantly publishes a message to a publisher. This class should be used as a
    context manager.
    """

    def __init__(self, publisher: Publisher, outgoing: Any) -> None:
        """
        :param publisher: The publisher to send messages on
        :param outgoing: The message to send
        """
        super().__init__("Constant Publisher")

        self._publisher = publisher
        self._outgoing = outgoing

    def run(self) -> None:
        while self.running:
            self._publisher.publish(self._outgoing)
            sleep(0.01)


_EQUALITY_MSG: TypeAlias = Float64MultiArray | JointState

T = TypeVar("T", bound=_EQUALITY_MSG)


def messages_equal(actual: T, expected: T) -> bool:
    """Checks if the given messages are equal"""
    is_equal = _get_equality_func(type(expected))
    return is_equal(actual, expected)


def expect_message(
    messages: "queue.Queue[T]",
    expected: T,
    max_tries: int = 30,
) -> None:
    """Continuously publishes the given message and checks for the
    desired result on the from_firmware queue.

    :param messages: A queue where messages are put
    :param expected: The expected value to come from messages
    :param max_tries: The maximum number of from_firmware messages
        to read before giving up
    :raises AssertionError: If the expected value was not received
    """
    messages.queue.clear()

    expected_count = 0
    msg = None

    for _ in range(max_tries):
        msg = messages.get(timeout=5)
        if messages_equal(msg, expected):
            expected_count += 1
            if expected_count >= 5:
                return

    fail_message = f"Messages failed to stabilize at value {expected}"
    if msg is not None:
        fail_message += f", latest value is {msg}"

    raise AssertionError(fail_message)


def publish_and_expect_message(
    publisher: Publisher,
    outgoing: Any,
    messages: "queue.Queue[_EQUALITY_MSG]",
    expected: _EQUALITY_MSG,
    max_tries: int = 30,
) -> None:
    """Continuously publishes the given message and checks for the desired result on the
    from_firmware queue.

    :param publisher: The publisher to publish the outgoing message to
    :param outgoing: The outgoing message to publish
    :param messages: The from_firmware message queue
    :param expected: The expected value to come from from_firmware
    :param max_tries: The maximum number of from_firmware messages to read
        before giving up
    """
    with ConstantPublisher(publisher, outgoing):
        expect_message(messages, expected, max_tries)


_EQUALITY_FUNC = Callable[[_EQUALITY_MSG, _EQUALITY_MSG], bool]

_EQUALITY_FUNCS: dict[type[_EQUALITY_MSG], _EQUALITY_FUNC] = {}


def _float64_multi_array(a: Float64MultiArray, b: Float64MultiArray) -> bool:
    return bool(a.data.tolist() == b.data.tolist())


def _joint_state(a: JointState, b: JointState) -> bool:
    def all_are_close(values: Iterable[tuple[float, float]]) -> bool:
        return all(math.isclose(val1, val2, rel_tol=0.001) for val1, val2 in values)

    return (
        a.name == b.name
        and all_are_close(zip(a.position, b.position, strict=False))
        and all_are_close(zip(a.velocity, b.velocity, strict=False))
        and all_are_close(zip(a.effort, b.effort, strict=False))
    )


_EQUALITY_FUNCS[Float64MultiArray] = _float64_multi_array

_EQUALITY_FUNCS[JointState] = _joint_state


def _get_equality_func(msg_type: type[_EQUALITY_MSG]) -> _EQUALITY_FUNC:
    try:
        return _EQUALITY_FUNCS[msg_type]
    except KeyError:
        # Use default equality testing
        return lambda a, b: cast(bool, a == b)


def _flush_queue(q: "queue.Queue[Any]") -> None:
    """Empties the queue of existing items"""
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            break
