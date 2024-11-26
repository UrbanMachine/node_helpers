from unittest.mock import MagicMock

from .conftest import ExampleNode


def test_destroy_callbacks(example_node: ExampleNode) -> None:
    on_destroy_callback = MagicMock()
    example_node.on_destroy(on_destroy_callback)
    example_node.destroy_node()

    assert on_destroy_callback.call_count == 1
