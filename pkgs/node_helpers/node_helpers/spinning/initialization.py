import logging
import sys
from collections.abc import Callable

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .executor import MultiThreadedStackTracedExecutor

DEFAULT_MULTITHREADED_THREAD_COUNT = 16
"""The default thread count should be thought of as "how many concurrent actions or
services should a node be able to handle. In the future, this could be configurable, or
a ScalableThreadedExecutor could be designed which dynamically increases or decreases
thread count.

This constant is also used for automated tests that use multi threaded executors.
"""


def create_spin_function(
    node_cls: type[Node], multi_threaded: bool = False
) -> Callable[[], None]:
    """Returns a function that, when called, will initialize ROS and spin the given node
    Usage Example:
    In node.py
    >>>main = create_spin_function(MyNode)

    In pyproject.toml
    >>>[tool.poetry.scripts]
    >>> my_node = "node:main"

    :param node_cls: The node class to initialize and spin
    :param multi_threaded: If True, the node will be spun with a multi-threaded executor
    :returns: A function that can be run to initialize the node
    """

    def spin_fn() -> None:
        # Match the rclpy Node.get_logger() format closely, which is of format:
        # [LOG_LEVEL] [UNIX_TIMESTAMP] [NODE_NAMESPACE.NODE_NAME]: the log message
        # In the case of python logs we also add the field '[FILENAME:LINE_NUMBER]:'
        node_name_placeholder = "unknown.node_name"
        log_fmt = (
            f"[%(levelname)s] [%(created)s] [{node_name_placeholder}] "
            "[%(filename)s:%(lineno)d]: %(message)s"
        )
        logging.basicConfig(level=logging.DEBUG, format=log_fmt)

        rclpy.init(args=sys.argv)
        node = node_cls()

        # Now that the node is initialized, we can re-configure the logger so that it
        # contains the now-known node name and namespace information
        node_namespace = node.get_namespace().replace("/", ".")[1:]
        name_and_namespace = f"{node_namespace}.{node.get_name()}"
        log_fmt = log_fmt.replace(node_name_placeholder, name_and_namespace)
        logging.basicConfig(level=logging.DEBUG, format=log_fmt, force=True)

        # Create the appropriate executor
        if multi_threaded:
            executor = MultiThreadedStackTracedExecutor(
                num_threads=DEFAULT_MULTITHREADED_THREAD_COUNT
            )
        else:
            executor = SingleThreadedExecutor()

        try:
            rclpy.spin(node, executor)
        except KeyboardInterrupt:
            node.get_logger().warning("Closing due to SIGINT")
        node.destroy_node()
        rclpy.shutdown()

    return spin_fn
