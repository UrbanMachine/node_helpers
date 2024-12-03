from enum import Enum
from typing import Any, TypeVar

from .base_parser import BaseParser

EnumType = TypeVar("EnumType", bound=Enum)


class EnumParser(BaseParser[str, EnumType]):
    def __init__(self, enum_type: type[EnumType]):
        super().__init__(str, enum_type)

    def can_parse(self, type_: Any) -> bool:
        return issubclass(type_, Enum)

    def from_config_type(self, config_value: str) -> EnumType:
        return self.parsed_type(config_value)

    def as_config_type(self, parsed_value: EnumType) -> str:
        return self.config_type(parsed_value.value)
