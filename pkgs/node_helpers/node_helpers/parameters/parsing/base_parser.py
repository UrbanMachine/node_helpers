from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from rclpy.parameter import Parameter

from .utils import CONFIG_TO_ROS_MAPPING, ConfigurationType

ParsedType = TypeVar("ParsedType")
"""This represents the type after the parser has received it from ROS"""


class BaseParser(ABC, Generic[ConfigurationType, ParsedType]):
    def __init__(
        self, config_type: type[ConfigurationType], parsed_type: type[ParsedType]
    ):
        """
        :param config_type: The type as seen in a yaml file,
        :param parsed_type: The type after parsing. For example, an enum, a Path, etc.
        """
        self.config_type: type[ConfigurationType] = config_type
        self.parsed_type: type[ParsedType] = parsed_type

    @property
    def ros_type(self) -> Parameter.Type:
        """Return the ROS enum of the configuration type for this parser"""
        return CONFIG_TO_ROS_MAPPING[self.config_type]

    def can_parse(self, type_: type) -> bool:
        """This method can be overridden to allow a parser to take on a broader scope"""
        return type_ == self.parsed_type

    @abstractmethod
    def from_config_type(self, config_value: ConfigurationType) -> ParsedType:
        pass

    @abstractmethod
    def as_config_type(self, parsed_value: ParsedType) -> ConfigurationType:
        pass
