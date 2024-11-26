import logging
from abc import abstractmethod
from concurrent.futures import Future
from typing import Generic, TypeVar

from rclpy.node import Node

MESSAGE = TypeVar("MESSAGE")
"""A ROS message to look for, when selecting prompt options"""

Choice = TypeVar("Choice", bound=object)


class BasePrompter(Generic[MESSAGE]):
    def __init__(self, node: Node):
        # Insert a callback that stops waiting on a breakpoint during closing time
        node.context.on_shutdown(self.disconnect)

    @abstractmethod
    def _add_request_for_message(
        self, choices: tuple[tuple[MESSAGE, Choice], ...]
    ) -> "Future[Choice]":
        """Return a future that will result in the choice a human chose"""

    @abstractmethod
    def describe_message(self, message: MESSAGE) -> str:
        """Takes a message and returns a human readable description"""

    @abstractmethod
    def connect(self) -> None:
        """Set up any ROS subscriptions, services, etc"""

    @abstractmethod
    def disconnect(self) -> None:
        """Clear the subscription and end the current future (if there is any)

        Expectations:
        1) Set any ongoing future(s) to a RuntimeError
        2) Destroy any ROS subscriptions, services, etc
        3) Ensure no new futures can be created
        """

    def choose_async(
        self, choices: tuple[tuple[MESSAGE, Choice], ...]
    ) -> "Future[Choice]":
        msg = "\n\t".join(f"{self.describe_message(j)}: {c}" for j, c in choices)
        logging.warning(f"BREAKPOINT- Choose an option:\n\t{msg}")
        return self._add_request_for_message(choices)

    def choose(
        self, choices: tuple[tuple[MESSAGE, Choice], ...], timeout: float | None = None
    ) -> Choice:
        """Prompt the user to pick an option"""
        return self.choose_async(choices).result(timeout=timeout)
