from collections.abc import Callable
from time import time

from rclpy.node import Node

from .errors import ExecutorNotSetError


class ValidatesReadinessMixin:
    _node: Node
    """A reference to the node that handles this system"""

    _has_been_called: bool = False
    """This returns true once the server has been called and validated to be ready"""

    def _validate_rpc_server_is_ready(
        self, wait_fn: Callable[[], bool], rpc_name: str
    ) -> None:
        """This function solves the problem of making sure an action server is ready
        and callable by a client before calling it.

        There are two situations where a server might not be callable:
        1) The server is not 'ready'. In these cases, the call will hang forever.
        2) The client side doesn't have an executor set. E.g., rclpy.spin() is not
           yet running, which is common when in a Node.__init__(). In this case, an RPC
           call will also hang forever.

        For case 1, this function will block indefinitely and periodically print logs.
        For case 2, this function will raise an ExecutorNotSet exception with an
        explanation.

        :param wait_fn: A function that will wait some time return true/false if
            the service or action is ready.
        :param rpc_name: The name of the service or action being used.
        :raises ExecutorNotSetError: When called in a nodes __init__.
        """

        if not self._has_been_called:
            if self._node.executor is None:
                raise ExecutorNotSetError(
                    f"The RPC '{self}' has been called before a node was assigned an "
                    f"executor! Did you try running an RPC call in the init of a node? "
                    f"Try using a single shot timer instead!"
                )

            start_time = time()
            while not wait_fn():
                elapsed = time() - start_time
                self._node.get_logger().error(
                    f"RPC server '{rpc_name}' is not ready yet after {elapsed} seconds!"
                )
            self._has_been_called = True
