from collections.abc import Generator
from time import sleep
from typing import Any

import pytest
from node_helpers.interaction import DashboardMenu, DashboardPrompter
from node_helpers.interaction.prompting.dashboard_prompter import InvalidPromptError
from node_helpers.nodes import HelpfulNode
from node_helpers.robust_rpc import RobustRPCException
from node_helpers.testing import set_up_node
from node_helpers.timing import TestingTimeout as Timeout
from node_helpers_msgs.msg import PromptOption
from node_helpers_msgs.srv import ChoosePromptOption

_OPTION_A = PromptOption(name="A", description="ayy lmao")
_OPTION_B = PromptOption(name="B", description="ayy lmao")
_OPTION_C = PromptOption(name="C", description="ayy lmao")
_OPTION_D = PromptOption(name="D", description="ayy lmao")


class PrompterNode(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("prompter", **kwargs)
        self.choose_option = self.create_robust_client(
            srv_type=ChoosePromptOption, srv_name=DashboardMenu.DEFAULT_OPTION_SERVICE
        )
        self.prompter = DashboardPrompter(self, DashboardMenu.DEFAULT_OPTION_SERVICE)
        self.prompter.connect()


@pytest.fixture()
def prompter_node() -> Generator[PrompterNode, None, None]:
    yield from set_up_node(
        PrompterNode, namespace="prompter", node_name="prompter", multi_threaded=True
    )


@pytest.mark.parametrize(
    ("options", "chosen_option", "expected_result"),
    (
        # Basic multi-option test
        (
            ((_OPTION_A, 1), (_OPTION_B, 2), (_OPTION_C, 3), (_OPTION_D, 4)),
            _OPTION_C,
            3,
        ),
        # Single option test
        (((_OPTION_A, "choice"),), _OPTION_A, "choice"),
    ),
)
def test_choose(
    prompter_node: PrompterNode,
    options: tuple[tuple[PromptOption, Any]],
    chosen_option: PromptOption,
    expected_result: Any,
) -> None:
    choice_future = prompter_node.prompter.choose_async(options)
    assert not choice_future.done()

    # Choose the option. This also tests that only the "name" is looked at, and that the
    # description does not matter.
    prompter_node.choose_option.call(ChoosePromptOption.Request(option=chosen_option))
    assert choice_future.done()
    assert choice_future.result() == expected_result


def test_choose_invalid_option(prompter_node: PrompterNode) -> None:
    # Test requesting an option when there are no prompts ongoing
    with pytest.raises(RobustRPCException.like(InvalidPromptError)):
        prompter_node.choose_option.call(ChoosePromptOption.Request(option=_OPTION_C))

    # Test requesting an option when the option isn't the one being requested for
    choice_future = prompter_node.prompter.choose_async(((_OPTION_C, "option c"),))
    with pytest.raises(RobustRPCException.like(InvalidPromptError)):
        prompter_node.choose_option.call(ChoosePromptOption.Request(option=_OPTION_A))

    assert not choice_future.done()

    # Now validate that getting the correct option _does_ work
    prompter_node.choose_option.call(ChoosePromptOption.Request(option=_OPTION_C))
    assert choice_future.result() == "option c"


def test_disconnect(prompter_node: PrompterNode) -> None:
    choice_future = prompter_node.prompter.choose_async(((_OPTION_A, 1),))

    # Assert initial state, which will all be torn down by the end
    assert prompter_node.choose_option.wait_for_service(10)
    assert prompter_node.choose_option.service_is_ready()
    assert prompter_node.prompter._ongoing_request is not None
    assert not choice_future.done()

    # Close the prompter
    prompter_node.prompter.disconnect()

    # The ongoing request should have been cancelled
    assert prompter_node.prompter._ongoing_request is None

    # The current future should have an exception set
    assert choice_future.done()
    with pytest.raises(RuntimeError):
        choice_future.result()

    # Wait for the service to be destroyed
    timeout = Timeout(5)
    while timeout and prompter_node.choose_option.service_is_ready():
        sleep(0.1)
    assert not prompter_node.choose_option.service_is_ready()


def test_connecting_and_disconnecting(prompter_node: PrompterNode) -> None:
    new_prompter = DashboardPrompter(
        prompter_node, DashboardMenu.DEFAULT_OPTION_SERVICE
    )
    assert new_prompter._service is None, "Service should not be created in the init"

    new_prompter.connect()
    assert new_prompter._service is not None, "Service should be created after connect"

    new_prompter.disconnect()
    assert new_prompter._service is None, "Service should be destroyed after disconnect"
