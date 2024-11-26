from collections.abc import Generator
from contextlib import contextmanager

from node_helpers_msgs.msg import PromptOption, UserPrompt

from node_helpers.nodes import HelpfulNode
from node_helpers.topics import LatchingPublisher

from ..prompting import DashboardPrompter
from .base_menu import BaseMenu


class DashboardMenu(BaseMenu[PromptOption]):
    """A menu controlled by buttons on a web UI"""

    DEFAULT_PROMPT_TOPIC = "/dashboard/prompts"
    DEFAULT_OPTION_SERVICE = "/dashboard/choose_option"

    def __init__(self, node: HelpfulNode):
        self._latching_prompt_publisher = LatchingPublisher(
            node, UserPrompt, topic=self.DEFAULT_PROMPT_TOPIC
        )
        self._dashboard_prompter = DashboardPrompter(
            node=node, srv_name=self.DEFAULT_OPTION_SERVICE
        )

        super().__init__(
            prompter=self._dashboard_prompter,
            publish_prompt=self._latching_prompt_publisher,
        )

    @contextmanager
    def connected(self) -> Generator["DashboardMenu", None, None]:
        self.connect()
        try:
            yield self
        finally:
            self.disconnect()

    @contextmanager
    def disconnected(self) -> Generator[None, None, None]:
        self.disconnect()
        try:
            yield
        finally:
            self.connect()

    def connect(self) -> None:
        self._dashboard_prompter.connect()

    def disconnect(self) -> None:
        """Relinquish control of the prompt topic, so another node can use it"""
        self._latching_prompt_publisher.clear_msg_state()
        self._dashboard_prompter.disconnect()
