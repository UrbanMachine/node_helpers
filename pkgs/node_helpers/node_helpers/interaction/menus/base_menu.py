from collections.abc import Callable
from concurrent.futures import CancelledError, Future
from functools import partial
from queue import Queue
from threading import Event
from typing import Any, Generic, TypeVar

from action_msgs.msg import GoalStatus
from node_helpers_msgs.msg import PromptOption, UserPrompt
from rclpy.action.client import ClientGoalHandle

from node_helpers.robust_rpc import RobustActionClient

from ..prompting import BasePrompter
from .prompt_metadata import BasicPromptMetadata, PromptMetadata

MESSAGE = TypeVar("MESSAGE")
"""What ros message to look for"""

PROMPT_PUBLISHER = Callable[[UserPrompt], None]
MENU_ITEMS = tuple[PromptOption, Callable[[], None]]
DEFAULT_CANCEL_OPTION = PromptOption(name="Cancel", description="❌")


class BaseMenu(Generic[MESSAGE]):
    """This class can take in any kind of Prompter and turn it into a menu"""

    def __init__(
        self, prompter: BasePrompter[MESSAGE], publish_prompt: PROMPT_PUBLISHER
    ):
        """
        :param prompter: The type of prompter to use for this menu
        :param publish_prompt: A callable to publish a prompt to the user, so they know
            what buttons can be pressed
        """

        self.publish_prompt = publish_prompt
        self._prompter = prompter

    def display_menu_async(
        self,
        *menu_items: MENU_ITEMS,
        help_message: str = "",
        metadata: PromptMetadata | None = None,
    ) -> "Future[Callable[[], None]]":
        metadata = metadata or BasicPromptMetadata()

        prompt_options = [o for o, _ in menu_items]

        # Show the options to the user
        self.publish_prompt(
            UserPrompt(
                options=prompt_options,
                help=help_message,
                type=metadata.type(),
                metadata=metadata.model_dump_json(),
            )
        )

        choice_future = self._prompter.choose_async(choices=menu_items)

        # Add a callback that will call the chosen callable when the future finishes
        def on_done(future: "Future[Callable[[], None]]") -> None:
            try:
                future.result()()
            except CancelledError:
                # If the future was cancelled, no need to call a callback
                pass

        choice_future.add_done_callback(on_done)
        return choice_future

    def display_menu(
        self,
        *menu_items: MENU_ITEMS,
        help_message: str = "",
        metadata: PromptMetadata | None = None,
    ) -> None:
        self.display_menu_async(
            *menu_items, help_message=help_message, metadata=metadata
        ).result()

    def run_user_cancellable_action(
        self,
        action: RobustActionClient,
        goal: Any,
        help_message: str = "",
        menu_items: tuple[MENU_ITEMS, ...] | None = None,
        metadata: PromptMetadata | None = None,
    ) -> tuple[Any, bool]:
        """Run an action while also checking for user input to see if the user wants
        the action cancelled. If no menu items are specified, a single option will be
        created to allow cancelling the action. if menu items are specified, then
        any of those menu items will cancel the action, but will also call their
        specified callbacks.

        :param action: The action to run
        :param goal: The goal to give the action
        :param help_message: The help message to display while the action runs
        :param menu_items: An optional list of menu items. All menu items when selected
            will cancel the current action, but will also call their specified callback.
        :param metadata: Specifies additional data for the prompt, enabling
            special features
        :returns: A tuple of (action result, bool "cancelled")
        """

        if menu_items is None:
            # If no menu items are set, default to having a simple "Cancel action" item.
            menu_items = ((DEFAULT_CANCEL_OPTION, lambda: None),)

        # Create an event that will be set either by the prompter or the action
        finished_or_canceled = Event()

        def wrap_callback(callback: Callable[[], None]) -> Callable[[], None]:
            """Wrap a callback to also set the finished_or_cancelled event"""

            def wrapper() -> None:
                finished_or_canceled.set()
                return callback()

            return wrapper

        handle: ClientGoalHandle
        with action.send_goal_as_context(goal=goal) as handle:
            # Augment each menu item to also set the 'finished_or_cancelled' event
            menu_items_that_also_cancel_action = [
                (text, wrap_callback(callback)) for text, callback in menu_items
            ]

            menu_future = self.display_menu_async(
                *menu_items_that_also_cancel_action,
                help_message=help_message,
                metadata=metadata,
            )
            handle.get_result_async().add_done_callback(
                lambda *args: finished_or_canceled.set()
            )

            # Now wait for the user to either cancel the request, or it to finish
            finished_or_canceled.wait()
            self.publish_prompt(UserPrompt(help="Finishing action..."))

        result = handle.get_result()
        canceled = menu_future.done() or result.status == GoalStatus.STATUS_CANCELED

        menu_future.cancel()
        return result, canceled

    def ask_yes_no(self, question: str, yes: str = "Yes", no: str = "No") -> bool:
        response: "Queue[bool]" = Queue()
        self.display_menu(
            (PromptOption(name=yes, description="👍"), partial(response.put, True)),
            (PromptOption(name=no, description="👎"), partial(response.put, False)),
            help_message=question,
        )
        return response.get_nowait()
