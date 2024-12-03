from pathlib import Path
from typing import Generic, TypeVar

from rclpy.serialization import deserialize_message

from .resource_paths import resource_path

MsgType = TypeVar("MsgType")


class MessageResource(Generic[MsgType]):
    """A helper class for loading serialized ROS messages"""

    def __init__(self, *paths: str | Path, msg_type: type[MsgType]):
        self.path = resource_path(*paths)
        self.msg_type = msg_type

    @property
    def msg(self) -> MsgType:
        msg_bytes = self.path.read_bytes()
        msg: MsgType = deserialize_message(msg_bytes, self.msg_type)
        return msg
