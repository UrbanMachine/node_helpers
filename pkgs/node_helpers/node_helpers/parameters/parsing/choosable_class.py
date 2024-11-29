"""This module needs a type hinting expert to help remove the type: ignore comments."""

import typing
from typing import Any, TypeVar

from node_helpers.parameters.choosable_object import Choosable

from .base_parser import BaseParser

ChoosableType = TypeVar("ChoosableType", bound=Choosable)


class ChoosableClassParser(BaseParser[str, type[Choosable]]):
    def __init__(self, for_chooseable_type: type[ChoosableType]):
        """A parser for a class that subclasses Choosable.
        :param for_chooseable_type: The type of the class that this parser will parse.

        For example,
        >>> ChoosableClassParser(for_choosable_type=type[SomeBaseChoosableClass])
        """
        super().__init__(str, for_chooseable_type)  # type: ignore

    def can_parse(self, type_: type[Any]) -> bool:
        """ChoosableTypes are intended to be specified as 'type[ChoosableClass]' in
        configuration. For that reason, type_ is always a type, and in order to figure
        out if the inside of type[THING] is a Choosable Type, we need to check if
        type_[0] is a subclass of Choosable.

        :param type_: The type to validate as a ChoosableType
        :return: True if type_ is a Choosable, False otherwise
        """
        try:
            unpacked_type: Any = self._unpack_type_from_type_wrapper(type_)
        except ValueError:
            return False

        return issubclass(unpacked_type, Choosable)

    def from_config_type(self, config_value: str) -> type[ChoosableType]:
        unpacked = self._unpack_type_from_type_wrapper(self.parsed_type)  # type: ignore
        return unpacked.get_registered_child_class(config_value)  # type: ignore

    def as_config_type(self, parsed_value: type[ChoosableType]) -> str:
        return parsed_value.get_registered_class_name()

    def _unpack_type_from_type_wrapper(
        self, type_: type[ChoosableType]
    ) -> type[ChoosableType]:
        """Unpack the choosable type type from a type wrapper.
        For example, if type_ is type[CoolChoosableClass], return CoolChoosableClass.

        :param type_: The type to unpack
        :return: The unpacked type, or None if it was not unpackable.
        :raises ValueError: If the type was not a type wrapper holding 1 type
        """
        if typing.get_origin(type_) is not type:
            raise ValueError

        type_args = typing.get_args(type_)
        if len(type_args) != 1:
            raise ValueError

        return type_args[0]  # type: ignore
