from types import NoneType

from .base_parser import BaseParser


class NoneParser(BaseParser[None, None]):  # type: ignore
    def __init__(self) -> None:
        super().__init__(NoneType, type[None])  # type: ignore

    def can_parse(self, type_: type) -> bool:
        return type_ is NoneType

    def from_config_type(self, config_value: None) -> None:
        return None

    def as_config_type(self, parsed_value: None) -> None:
        return None
