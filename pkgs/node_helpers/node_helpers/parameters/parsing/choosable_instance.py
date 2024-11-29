from typing import Any, TypeVar

from node_helpers.parameters.choosable_object import Choosable

from .base_parser import BaseParser

ChoosableInstance = TypeVar("ChoosableInstance", bound=Choosable)


class ChoosableInstanceParser(BaseParser[str, ChoosableInstance]):
    def __init__(self, for_chooseable_instance: type[ChoosableInstance]):
        """A parser for retrieving an instance of a  class that subclasses Choosable

        For example,
        >>> ChoosableInstanceParser(for_chooseable_instance=SomeBaseChoosableClass)

        :param for_chooseable_instance: The type of the class that this parser will
            parse. It is _not_ wrapped in a type[], unlike the ChoosableClassParser.
        """
        super().__init__(str, for_chooseable_instance)

    def can_parse(self, type_: type[Any]) -> bool:
        return issubclass(type_, Choosable)

    def from_config_type(self, config_value: str) -> ChoosableInstance:
        return self.parsed_type.get_registered_instance(config_value)

    def as_config_type(self, parsed_value: ChoosableInstance) -> str:
        return parsed_value.get_registered_instance_name()
