from collections.abc import Generator
from typing import Any

import pytest
from node_helpers.nodes import HelpfulNode
from node_helpers.testing import set_up_node


class ExampleNode(HelpfulNode):
    def __init__(self, **kwargs: Any):
        super().__init__("example_async_node_client", **kwargs)


@pytest.fixture()
def example_node() -> Generator[ExampleNode, None, None]:
    yield from set_up_node(
        node_class=ExampleNode,
        namespace="async_adapter",
        node_name="example_async_node",
    )
