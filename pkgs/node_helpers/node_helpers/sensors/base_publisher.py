import logging
from abc import ABC, abstractmethod
from threading import RLock
from typing import Any, Generic, TypeVar

from builtin_interfaces.msg import Time
from pydantic import BaseModel
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from node_helpers import markers
from node_helpers.tf import timestamps
from node_helpers.timing import ttl_cached

from .typing import SensorProtocol

SENSOR_MSG = TypeVar("SENSOR_MSG", bound=SensorProtocol)
SENSOR_VALUE = TypeVar("SENSOR_VALUE", bound=Any)
PARAMETERS = TypeVar("PARAMETERS", bound="BaseSensorPublisher.Parameters")

DEFAULT_SENSORS_VIS_TOPIC = "/debug/sensors"
WRAPPED_FN = TypeVar("WRAPPED_FN")


class BaseSensorPublisher(ABC, Generic[SENSOR_MSG, SENSOR_VALUE, PARAMETERS]):
    """The basic sensor publisher structure.
    It standardizes sensor publishing by assigning a default QoS, standardizing the
    visualization callbacks, and standardizing sensor message structure.
    """

    class Parameters(BaseModel):
        frame_id: str
        """The frame ID wherein the sensor is centered upon the origin."""

        sensor_topic: str
        """The topic to publish sensor values to."""

        vis_topic: str = DEFAULT_SENSORS_VIS_TOPIC
        """The topic to publish visualization markers to."""

        sensor_publishing_max_hz: float = 0.0
        """Optionally throttle the sensor publishing rate. 0.0 means no throttle."""

        vis_publishing_max_hz: float = 0.0
        """Optionally throttle the visualization publishing rate. 0.0 means no throttle.
        """

    def __init__(
        self,
        node: Node,
        msg_type: type[SENSOR_MSG],
        parameters: PARAMETERS,
        sensor_qos: QoSProfile = qos_profile_sensor_data,
        vis_qos: QoSProfile = qos_profile_sensor_data,
    ):
        """
        :param node: The node to use for publishing
        :param msg_type: The sensor message type
        :param parameters: The parameters for the sensor publisher
        :param sensor_qos: The QoS for sensor publishing
        :param vis_qos: The QoS for visualization publishing
        """
        self._params = parameters
        self._sensor_topic = self._params.sensor_topic
        self._msg_type = msg_type
        self._clock = node.get_clock()
        self._sensor_publisher = node.create_publisher(
            msg_type,
            self._sensor_topic,
            sensor_qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._visualization_publisher = node.create_publisher(
            MarkerArray,
            self._params.vis_topic,
            vis_qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._last_published_time: Time = Time()
        """For ensuring there's no out-of-order publishing"""

        self.frame_id = self._params.frame_id

        self._publish_sensor_lock = RLock()
        self.publish_sensor = self._maybe_throttle_function(  # type: ignore
            self._params.sensor_publishing_max_hz, self.publish_sensor
        )
        self._publish_rviz_markers = self._maybe_throttle_function(  # type: ignore
            self._params.vis_publishing_max_hz, self._publish_rviz_markers
        )

    def _maybe_throttle_function(self, max_hz: float, fn: WRAPPED_FN) -> WRAPPED_FN:
        """Optionally wrap a function with a rate limiter"""
        if max_hz <= 0:
            return fn

        logging.info(
            f"Throttling sensor '{self.frame_id}' {fn.__name__} to {max_hz}hz"  # type: ignore
        )
        return ttl_cached(1 / max_hz)(fn)  # type: ignore

    @property
    def marker_namespace(self) -> str:
        return f"{self.frame_id}.{self._sensor_topic}"

    def publish_value(
        self, sensor_value: SENSOR_VALUE, stamp: Time | None = None
    ) -> None:
        """Helper for publishing a stamped sensor message

        :param sensor_value: The Sensor.value portion of the message
        :param stamp: If none, clock.now() will be used for the header.
        """

        # Since old timestamps can be rejected, we hold the lock so that we can create
        # a new stamp (when needed) and publish it before another thread possibly can.
        with self._publish_sensor_lock:
            msg = self._msg_type(
                header=Header(
                    frame_id=self.frame_id, stamp=stamp or self._clock.now().to_msg()
                ),
                value=sensor_value,
            )
            self.publish_sensor(msg)

    def publish_sensor(self, sensor_msg: SENSOR_MSG) -> None:
        # Ensure the message is filled out
        if sensor_msg.header.stamp == Time():
            raise ValueError("A sensor message cannot have an empty timestamp!")
        if sensor_msg.header.frame_id != self.frame_id:
            raise ValueError(
                "Until a need arises, it's not allowed to publish a sensor message with"
                " a frame_id different than the publishers assigned frame_id!"
            )

        with self._publish_sensor_lock:
            # Ensure that we're not publishing out of order
            if timestamps.is_older(sensor_msg.header.stamp, self._last_published_time):
                logging.warning(
                    f"The buffer was told to publish an out of order message! This is "
                    f"only okay during initialization, when initial values are being "
                    f"set while upstream messages might be arriving. Sensor: "
                    f"'{self.frame_id}'. "
                    f"Message: {sensor_msg}, Latest Time: {self._last_published_time}"
                )

            self._sensor_publisher.publish(sensor_msg)
            self._last_published_time = sensor_msg.header.stamp
            self._publish_rviz_markers(sensor_msg)

    def _publish_rviz_markers(self, sensor_msg: SENSOR_MSG) -> None:
        """Publish a list of rviz markers to the visualization topic"""
        # Create visualizations for this sensor
        rviz_markers = self.to_rviz_msg(sensor_msg)
        # Clean up the markers so that each marker has the same header as the sensor msg
        # This is pretty opinionated, and could be a target for change in the future.
        for marker in rviz_markers:
            if marker.header == Header():
                marker.header = sensor_msg.header
            elif not (marker.header.frame_id != "" and marker.header.stamp != Time()):
                raise ValueError(
                    "If a marker header already has has either a frame_id or a "
                    "time stamp, then it must be fully defined!"
                )
        namespaced_marker_array = markers.ascending_id_marker_array(
            rviz_markers, marker_namespace=self.marker_namespace
        )

        self._visualization_publisher.publish(namespaced_marker_array)

    @abstractmethod
    def to_rviz_msg(self, msg: SENSOR_MSG) -> list[Marker]:
        """This method should be able to take a sensor msg and convert it to markers"""
        raise NotImplementedError
