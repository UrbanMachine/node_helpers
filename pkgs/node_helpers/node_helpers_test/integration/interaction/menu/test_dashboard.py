import json
from collections.abc import Generator
from queue import Queue
from threading import Event
from time import sleep
from typing import Any
from unittest import mock

import pytest
from action_msgs.msg import GoalStatus
from node_helpers.interaction import DashboardMenu
from node_helpers.interaction.menus.base_menu import DEFAULT_CANCEL_OPTION
from node_helpers.nodes import HelpfulNode
from node_helpers.testing import DynamicContextThread, set_up_node
from node_helpers.timing import TestingTimeout as Timeout
from node_helpers_msgs.action import RobustActionExample
from node_helpers_msgs.action._robust_action_example import (
    RobustActionExample_GetResult_Response,
)
from node_helpers_msgs.msg import PromptOption, UserPrompt
from node_helpers_msgs.srv import ChoosePromptOption
from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle

from .conftest import MenuClient


class SimpleActionNode(HelpfulNode):
    """Hosts an arbitrary action server and client"""

    EXPECTED_ACTION_RESULT_DATA = "cool-data"

    def __init__(self, **kwargs: Any):
        super().__init__(node_name="simple_node", **kwargs)
        self.action_continue_event = Event()
        self.action_cancel_requested = Event()
        """When set, the action call will continue and finish"""

        self.server = self.create_robust_action_server(
            RobustActionExample,
            "cool_action",
            execute_callback=self.execute_action,
            cancel_callback=lambda *args: CancelResponse.ACCEPT,
        )
        self.client = self.create_robust_action_client(
            RobustActionExample, "cool_action"
        )

    def execute_action(self, goal: ServerGoalHandle) -> RobustActionExample.Result:
        while not self.action_continue_event.is_set():
            if goal.is_cancel_requested:
                self.action_cancel_requested.set()
                goal.canceled()
                return RobustActionExample.Result()
            sleep(0.1)

        goal.succeed()
        return RobustActionExample.Result(data=self.EXPECTED_ACTION_RESULT_DATA)


@pytest.fixture()
def simple_action_node() -> Generator[SimpleActionNode, None, None]:
    yield from set_up_node(
        node_class=SimpleActionNode,
        namespace="",
        node_name="simple_node",
        multi_threaded=True,
    )


@pytest.fixture()
def menu(menu_client: MenuClient) -> DashboardMenu:
    menu = DashboardMenu(node=menu_client)
    menu.connect()
    return menu


def test_display_menu(menu_client: MenuClient, menu: DashboardMenu) -> None:
    very_helpful_message = "dingus"

    expected_prompt = UserPrompt(
        options=[
            PromptOption(
                name="do an A thing",
                description="A",
            ),
            PromptOption(
                name="do a B thing",
                description="B",
            ),
        ],
        help=very_helpful_message,
        metadata=json.dumps({}),
    )

    prompt_publisher: "Queue[UserPrompt]" = Queue()
    selected_a = mock.Mock()
    selected_b = mock.Mock()

    menu.publish_prompt = prompt_publisher.put

    def display_menu() -> None:
        menu.display_menu(
            (expected_prompt.options[0], selected_a),
            (expected_prompt.options[1], selected_b),
            help_message=very_helpful_message,
        )

    with DynamicContextThread(target=display_menu):
        wait_for_menu_readyness(menu)

        assert selected_b.call_count == 0
        menu_client.choose_option.call(
            ChoosePromptOption.Request(option=expected_prompt.options[1])
        )

    assert selected_a.call_count == 0
    assert selected_b.call_count == 1
    assert prompt_publisher.get() == expected_prompt


@pytest.mark.parametrize("user_cancelled", (True, False))
def test_run_cancellable_action(
    simple_action_node: SimpleActionNode,
    menu_client: MenuClient,
    user_cancelled: bool,
    menu: DashboardMenu,
) -> None:
    prompt_publisher: "Queue[UserPrompt]" = Queue()
    menu.publish_prompt = prompt_publisher.put

    output: "Queue[tuple[RobustActionExample_GetResult_Response, bool]]" = Queue()

    def run_action() -> None:
        output.put(
            menu.run_user_cancellable_action(
                simple_action_node.client,
                RobustActionExample.Goal(),
            )
        )

    with DynamicContextThread(run_action):
        assert (
            len(prompt_publisher.get(timeout=5).options) == 1
        ), "The user should be prompted to cancel!"
        assert output.qsize() == 0
        assert prompt_publisher.qsize() == 0

        if user_cancelled:
            wait_for_menu_readyness(menu)

            # Select a choice
            menu_client.choose_option.call(
                ChoosePromptOption.Request(option=DEFAULT_CANCEL_OPTION)
            )

            # Wait for the action to be cancelled
            simple_action_node.action_cancel_requested.wait(5)

        # Allow the action to continue
        simple_action_node.action_continue_event.set()
        result, cancelled = output.get(timeout=15)

    assert cancelled is user_cancelled
    assert prompt_publisher.qsize() == 1, "There should be a 'finishing action' log!"

    if user_cancelled:
        assert result.status == GoalStatus.STATUS_CANCELED
        assert result.result.data == ""
    else:
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.data == simple_action_node.EXPECTED_ACTION_RESULT_DATA


def test_run_cancellable_action_with_custom_menu_items(
    simple_action_node: SimpleActionNode, menu_client: MenuClient, menu: DashboardMenu
) -> None:
    """Test that when using custom menu items, the callbacks are processed as expected
    and cancellation still occurs"""
    prompt_publisher: "Queue[UserPrompt]" = Queue()
    menu.publish_prompt = prompt_publisher.put

    output: "Queue[tuple[RobustActionExample_GetResult_Response, bool]]" = Queue()

    option_1_event = Event()
    option_2_event = Event()
    option_3_event = Event()
    option_1 = PromptOption(name="1")
    option_2 = PromptOption(name="2")
    option_3 = PromptOption(name="3")

    def run_action() -> None:
        output.put(
            menu.run_user_cancellable_action(
                simple_action_node.client,
                RobustActionExample.Goal(),
                menu_items=(
                    (option_1, option_1_event.set),
                    (option_2, option_2_event.set),
                    (option_3, option_3_event.set),
                ),
            )
        )

    with DynamicContextThread(run_action):
        wait_for_menu_readyness(menu)

        # Select the second option out of the three
        menu_client.choose_option.call(ChoosePromptOption.Request(option=option_2))

        # Wait for the action to be cancelled
        simple_action_node.action_cancel_requested.wait(5)

        # Allow the action to continue
        simple_action_node.action_continue_event.set()
        result, cancelled = output.get(timeout=15)

    # Validate that even though some arbitrary option got picked, it was cancelled
    assert cancelled
    # Validate the expected option was picked
    assert not option_1_event.is_set()
    assert option_2_event.is_set()
    assert not option_3_event.is_set()
    # Validate the prompt included three options
    assert len(prompt_publisher.get().options) == 3


def wait_for_menu_readyness(menu: DashboardMenu) -> None:
    """Wait for the menu to be ready for user input"""
    timeout = Timeout(5)
    while menu._prompter._ongoing_request is None and timeout:  # type: ignore
        sleep(0.05)
