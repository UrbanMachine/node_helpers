import logging
from abc import ABC
from typing import Generic, TypeVar, cast

from builtin_interfaces.msg import Time
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time as RclpyTime

from node_helpers.pubsub import Topic
from node_helpers.tf import timestamps
from node_helpers.timing import Timeout

from .typing import SensorProtocol

SENSOR_MSG = TypeVar("SENSOR_MSG", bound=SensorProtocol)


class BaseSensorBuffer(ABC, Generic[SENSOR_MSG]):
    """A sensor buffer is simple: It subscribes to a sensor topic, and holds the latest
    value for the user.
    """

    def __init__(
        self,
        node: Node,
        msg_type: SENSOR_MSG,
        sensor_topic: str,
        sensor_qos: QoSProfile = qos_profile_sensor_data,
        callback_group: CallbackGroup | None = None,
    ):
        super().__init__()
        # Topics
        self.on_value_change = Topic[SENSOR_MSG]()
        """Called when a reading has changed from a previous value"""
        self.on_receive = Topic[SENSOR_MSG]()
        """Called whenever a new reading is received"""

        self._latest_reading: SENSOR_MSG | None = None
        node.create_subscription(
            msg_type=msg_type,
            topic=sensor_topic,
            callback=self._on_receive,
            qos_profile=sensor_qos,
            callback_group=callback_group,
        )

    def _on_receive(self, msg: SENSOR_MSG) -> None:
        """Called whenever a sensor reading is received"""
        changed = (
            self._latest_reading is None or msg.value != self._latest_reading.value
        )

        # Verify if the message is newer than the latest one
        if self._latest_reading is not None and timestamps.is_older(
            msg.header.stamp,
            self._latest_reading.header.stamp,
        ):
            logging.error(
                f"Refusing to receive out-of-order message for {msg.header.frame_id}!"
            )
            return

        self._latest_reading = msg

        # Alert users of the callback API
        if changed:
            self.on_value_change.publish(self._latest_reading)
        self.on_receive.publish(self._latest_reading)

    def get(self, after: Time = None, timeout: float | None = None) -> SENSOR_MSG:
        """Get a sensor message from the buffer

        :param after: Get a sensor reading at or after a given time. If None, the latest
            reading in the buffer will be returned. If 0, the latest_reading will be
            checked and returned if it fits the 'after' criteria, otherwise a
            TimeoutError will immediately be raised.
        :param timeout: If None, block until the 'after' criteria is met. If 0, check
            instantaneously if the 'after' criteria is met, and if not fail immediately.
            If greater than 0, wait that amount of seconds until the 'after' criteria
            is met.
        :raises RuntimeError: In impossible situations
        :return: The sensor message
        """

        # First, check if the latest value already in the buffer matches the requirement
        after = RclpyTime.from_msg(after) if after is not None else None
        if self._latest_reading is not None and (
            after is None
            or RclpyTime.from_msg(self._latest_reading.header.stamp) > after
        ):
            return self._latest_reading

        on_receive = self.on_receive.subscribe_as_event()
        check_timeout = (
            Timeout(timeout, raise_error=True) if timeout is not None else True
        )
        while check_timeout:
            if not on_receive.wait(timeout=timeout or 10.0):
                logging.warning(
                    f"{self.__class__.__name__}.get() is taking longer than"
                    f" expected..."
                )
                continue

            new_msg = cast(SENSOR_MSG, self._latest_reading)
            if after is not None and RclpyTime.from_msg(new_msg.header.stamp) < after:
                # This message is still older than the requested 'after' value
                continue

            return new_msg

        raise RuntimeError("This function should have returned a value!")
