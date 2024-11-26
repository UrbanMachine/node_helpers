from collections.abc import Generator
from typing import Any

import pytest
from node_helpers.interaction import DashboardMenu
from node_helpers.testing import (  # noqa: F401
    NodeForTesting,
    each_test_setup_teardown,
    set_up_node,
)
from node_helpers_msgs.srv import ChoosePromptOption


class MenuClient(NodeForTesting):
    def __init__(self, **kwargs: Any):
        super().__init__("menu_client", **kwargs)

        self.choose_option = self.create_robust_client(
            srv_type=ChoosePromptOption, srv_name=DashboardMenu.DEFAULT_OPTION_SERVICE
        )


@pytest.fixture()
def menu_client() -> Generator[MenuClient, None, None]:
    yield from set_up_node(
        node_class=MenuClient,
        namespace="",
        node_name="menu_client",
        multi_threaded=True,
    )
