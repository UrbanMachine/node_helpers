from typing import cast

from rclpy import Future
from rclpy.client import Client
from rclpy.node import Node, SrvTypeRequest

from ._readiness import ValidatesReadinessMixin
from ._wrappers import patch_future_to_raise_exception


class RobustServiceClient(Client, ValidatesReadinessMixin):
    """Wraps an rclpy.Client so that any exceptions raised by the remote server are
    raised in the client, when calling call_async().result() or call()
    """

    @classmethod
    def from_client(cls, client: Client, node: Node) -> "RobustServiceClient":
        """Wrap an instantiated client object in-place as a RobustServiceClient.

        Unlike the RobustActionClient, it was impossible to subclass the Client and
        patch rclpy to automatically create a RobustServiceClient. In place of that, the
        creation of the RobustServiceClient is done by monkeypatching the `__class__`
        attribute, in the from_client method.

        :param client: The client to wrap with the RobustServiceClient interface
        :param node: The node that is creating this client. Currently this is only used
            by the ValidatesReadinessMixin to check if there is an executor present.
        :returns: The client, but as a RobustServiceClient
        """
        client.__class__ = RobustServiceClient

        # Patch in the 'node' attribute to fulfill the ValidatesReadinessMixin needs
        client._node = node  # noqa: SLF001

        return client  # type: ignore

    def call_async(self, request: SrvTypeRequest) -> Future:
        """Patch the future so that when result() is called it raises remote errors"""
        self._validate_rpc_server_is_ready(
            wait_fn=lambda: cast(bool, self.wait_for_service(10)),
            rpc_name=self.srv_name,
        )

        future = super().call_async(request)

        patch_future_to_raise_exception(future=future, parse_as_action=False)
        return future
