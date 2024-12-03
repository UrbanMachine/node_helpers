import logging
from concurrent.futures import Future, InvalidStateError
from dataclasses import dataclass, field
from threading import RLock
from typing import Any

from node_helpers_msgs.msg import PromptOption
from node_helpers_msgs.srv import ChoosePromptOption
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.service import Service

from node_helpers.nodes import HelpfulNode

from .base_prompter import BasePrompter, Choice


class InvalidPromptError(Exception):
    pass


@dataclass
class _Request:
    choices: dict[str, object]
    """Store the PromptOption.name -> Choice relationship"""

    future: Future[Any] = field(default_factory=Future)


class DashboardPrompter(BasePrompter[PromptOption]):
    def __init__(self, node: HelpfulNode, srv_name: str):
        super().__init__(node)
        self._node = node
        self._service: Service | None = None
        self._srv_name: str = srv_name

        self._request_lock = RLock()
        self._ongoing_request: _Request | None = None

    def _on_user_chooses_option(
        self, request: ChoosePromptOption.Request, response: ChoosePromptOption.Response
    ) -> ChoosePromptOption.Response:
        """Called by a service, this means a user has chosen some menu option"""
        logging.info(
            f"User has selected the option {self.describe_message(request.option)}!"
        )

        option_name = request.option.name
        with self._request_lock:
            if self._ongoing_request is None:
                raise InvalidPromptError(
                    f"There's no ongoing prompt right now! Received: {option_name}"
                )

            try:
                choice = self._ongoing_request.choices[option_name]
            except KeyError as ex:
                raise InvalidPromptError(
                    f"The option '{option_name}' is not currently being prompted for!"
                ) from ex
            else:
                self._ongoing_request.future.set_result(choice)
                self._ongoing_request = None
        return response

    def _add_request_for_message(
        self, choices: tuple[tuple[PromptOption, Choice], ...]
    ) -> Future[Choice]:
        with self._request_lock:
            if self._ongoing_request is not None:
                # Mark the current future as cancelled
                self._ongoing_request.future.cancel()
            future: Future[Choice] = Future()
            self._ongoing_request = _Request(
                choices={o.name: c for o, c in choices}, future=future
            )
            return future

    def describe_message(self, message: PromptOption) -> str:
        """Detailed descriptions don't make much sense for PromptOptions"""
        return f"Option(name='{message.name}')"

    def connect(self) -> None:
        """A context for relinquishing control upon exit"""
        if self._service is None:
            self._service = self._node.create_robust_service(
                ChoosePromptOption,
                self._srv_name,
                self._on_user_chooses_option,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

    def disconnect(self) -> None:
        # Error out ongoing requests
        with self._request_lock:
            if self._service:
                self._node.destroy_service(self._service)
                self._service = None

            if self._ongoing_request:
                msg = "A system exit was requested while waiting for user input!"
                try:
                    self._ongoing_request.future.set_exception(RuntimeError(msg))
                except InvalidStateError:
                    pass
                self._ongoing_request = None
