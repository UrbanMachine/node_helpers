from typing import Any, Protocol

from std_msgs.msg import Header


class SensorProtocol(Protocol):
    """A sensor message will always have a header, and some value.

    The header.frame_id will be a TF where the sensor is centered upon the origin.
    The header.stamp is the timestamp of when that sensor reading was taken.
    """

    header: Header
    value: Any

    def __init__(self, header: Header, value: Any): ...
