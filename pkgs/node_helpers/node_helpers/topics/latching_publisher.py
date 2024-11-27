from typing import Generic, TypeVar

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from node_helpers.qos import qos_latching

T = TypeVar("T")


class LatchingPublisher(Generic[T]):
    """Publishes a message using QoS behavior similar to ROS1's latching QoS.
    The latest provided message is also routinely republished to ensure that
    improperly configured subscribers (like roslibjs) will still receive
    messages.
    """

    def __init__(
        self,
        node: Node,
        msg_type: type[T],
        topic: str,
        republish_delay: float = 1.0,
        *,
        callback_group: CallbackGroup | None = None,
    ):
        self._publisher = node.create_publisher(
            msg_type, topic, qos_profile=qos_latching, callback_group=callback_group
        )
        self._last_msg: T | None = None

        # Routinely republish the message. This shouldn't be necessary since
        # these topics are durable and transient local, but roslibjs will
        # improperly configure its subscribers if it subscribes before this
        # node has published. Sending routinely ensures that roslibjs will see
        # messages on this topic even in that case.
        node.create_timer(republish_delay, callback=self._republish)

    def __call__(self, msg: T) -> None:
        self._last_msg = msg
        self._publisher.publish(msg)

    def clear_msg_state(self) -> None:
        """Used for deferring control of the latching publisher to another node"""
        self._last_msg = None

    def _republish(self) -> None:
        if self._last_msg is not None:
            self._publisher.publish(self._last_msg)
