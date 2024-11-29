from typing import Generic

from .base_parser import BaseParser
from .utils import ConfigurationType


class PassthroughParser(
    BaseParser[ConfigurationType, ConfigurationType], Generic[ConfigurationType]
):
    def from_config_type(self, config_value: ConfigurationType) -> ConfigurationType:
        return config_value

    def as_config_type(self, parsed_value: ConfigurationType) -> ConfigurationType:
        return parsed_value
