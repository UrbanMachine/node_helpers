from pathlib import Path

from .base_parser import BaseParser


class PathParser(BaseParser[str, Path]):
    def __init__(self) -> None:
        super().__init__(str, Path)

    def can_parse(self, type_: type) -> bool:
        return issubclass(type_, Path)

    def from_config_type(self, config_value: str) -> Path:
        return self.parsed_type(config_value)

    def as_config_type(self, parsed_value: Path) -> str:
        return self.config_type(parsed_value)
