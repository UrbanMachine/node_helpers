from collections.abc import Callable
from typing import Any, cast

from rclpy.node import Node


class DestroyCallbacksMixin:
    _destroy_callbacks: list[Callable[[], Any]]

    def on_destroy(self, callback: Callable[[], Any]) -> None:
        """Registers a callback that the node will call when destroyed. Useful for
        cleaning up resources. Multiple callbacks can be registered.

        :param callback: The function to call
        """
        self._get_destroy_callbacks().append(callback)

    def destroy_node(self) -> None:
        """Overrides Node.destroy_node() to call registered callbacks"""
        for callback in self._get_destroy_callbacks():
            callback()
        cast(Node, super()).destroy_node()

    def _get_destroy_callbacks(self) -> list[Callable[[], Any]]:
        """Lazily creates the attribute that holds destroy callbacks. We do this to
        avoid having to deal with __init__ methods when using multiple inheritance.

        :return: The list of callbacks
        """
        try:
            return self._destroy_callbacks
        except AttributeError:
            self._destroy_callbacks = []
            return self._destroy_callbacks
