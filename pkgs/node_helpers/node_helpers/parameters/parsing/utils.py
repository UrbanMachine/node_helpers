from types import NoneType
from typing import TypeVar

from rclpy.parameter import Parameter

CONFIG_TO_ROS_MAPPING = {
    NoneType: Parameter.Type.NOT_SET,
    bool: Parameter.Type.BOOL,
    int: Parameter.Type.INTEGER,
    float: Parameter.Type.DOUBLE,
    str: Parameter.Type.STRING,
    list[bytes]: Parameter.Type.BYTE_ARRAY,
    list[bool]: Parameter.Type.BOOL_ARRAY,
    list[int]: Parameter.Type.INTEGER_ARRAY,
    list[float]: Parameter.Type.DOUBLE_ARRAY,
    list[str]: Parameter.Type.STRING_ARRAY,
}
"""This is a nice wrapper to allow python types to map to the ROS parameter API types"""

ConfigurationType = TypeVar(
    "ConfigurationType",
    type[None],
    bool,
    int,
    float,
    str,
    bytes,
    list[bool],
    list[bytes],
    list[int],
    list[float],
    list[str],
)
"""All supported ROS parameter types"""

ParsableType = TypeVar("ParsableType")
