from typing import Any

from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.service import Service

from ._wrappers import (
    ActionCallback,
    ServiceCallback,
    wrap_action_callback,
    wrap_service_callback,
)
from .action_client import RobustActionClient
from .action_server import RobustActionServer
from .schema import validate_robust_message
from .service_client import RobustServiceClient
from .typing import RobustActionMsg, RobustServiceMsg


class RobustRPCMixin:
    """This mixin adds methods for creating service/action clients and servers.

    When the client and server is used on both sides, then error messages raised by
    the server will be caught, passed in the response message, and then re-raised on the
    client side.

    The client will raise an error of the same name that subclasses RobustRPCException,
    which has the properties error_name, error_description, and message. The message
    holds the message object that caused the exception to be raised.
    """

    def create_robust_client(
        self: Node, srv_type: type[RobustServiceMsg], srv_name: str, **kwargs: Any
    ) -> RobustServiceClient:
        validate_robust_message(srv_type.Response)
        client = self.create_client(srv_type=srv_type, srv_name=srv_name, **kwargs)
        return RobustServiceClient.from_client(client, node=self)

    def create_robust_service(
        self: Node,
        srv_type: type[RobustServiceMsg],
        srv_name: str,
        callback: ServiceCallback,
        **kwargs: Any,
    ) -> Service:
        validate_robust_message(srv_type.Response)
        return self.create_service(
            srv_type=srv_type,
            srv_name=srv_name,
            callback=wrap_service_callback(callback=callback, srv_name=srv_name),
            **kwargs,
        )

    def create_robust_action_client(
        self: Node,
        action_type: type[RobustActionMsg],
        action_name: str,
        *,
        outside_station: bool = False,
        **kwargs: Any,
    ) -> RobustActionClient:
        validate_robust_message(action_type.Result)
        action_name = stationize_topic(
            action_name, station_from_namespace(self), outside_station
        )
        return RobustActionClient(self, action_type, action_name, **kwargs)

    def create_robust_action_server(
        self: Node,
        action_type: type[RobustActionMsg],
        action_name: str,
        execute_callback: ActionCallback,
        result_timeout: float | None = None,
        outside_station: bool = False,
        **kwargs: Any,
    ) -> ActionServer:
        validate_robust_message(action_type.Result)
        action_name = stationize_topic(
            action_name, station_from_namespace(self), outside_station
        )

        return RobustActionServer(
            self,
            action_type=action_type,
            action_name=action_name,
            execute_callback=wrap_action_callback(
                callback=execute_callback,
                action_name=action_name,
                result_type=action_type.Result,
            ),
            # By default, significantly reduce the result_timeout as compared to the
            # default rclpy behavior. This is done because the default behavior keeps a
            # cache of the last FIFTEEN MINUTES (!) worth of action call results, which
            # can lead to significant (60-80%) slowdowns in action call times.
            result_timeout=result_timeout or 10,
            **kwargs,
        )
