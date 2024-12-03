from typing import Any

from .base_parser import BaseParser
from .choosable_class import ChoosableClassParser
from .choosable_instance import ChoosableInstanceParser
from .enum import EnumParser
from .none import NoneParser
from .passthrough import PassthroughParser
from .path import PathParser
from .utils import CONFIG_TO_ROS_MAPPING, ConfigurationType, ParsableType


def get_parser(for_parseable_type: type[ParsableType]) -> BaseParser[Any, ParsableType]:
    """Map all the supported declarable types to their respective parsers

    :param for_parseable_type: Either a ROS Parameter type (e.g. int, float, str) or a
        custom type with a custom parser (e.g. a custom Enum, a Path, etc).
    :return: The parser that can convert between the python ros type and the custom type
    :raises RuntimeError: If the requested type isn't yet supported.
    """

    parsers: list[BaseParser[Any, Any]] = [
        # These are custom types, where the final type is not equivalent to the
        # type written in yaml / returned by ROS.
        NoneParser(),
        PathParser(),
        EnumParser(for_parseable_type),
        ChoosableClassParser(for_parseable_type),  # type: ignore
        ChoosableInstanceParser(for_parseable_type),
        # These are passthrough types, where the ROS configuration type is equal to the
        # final "parsed" type
        PassthroughParser(bool, bool),
        PassthroughParser(int, int),
        PassthroughParser(float, float),
        PassthroughParser(str, str),
        PassthroughParser(list[bytes], list[bytes]),
        PassthroughParser(list[bool], list[bool]),
        PassthroughParser(list[int], list[int]),
        PassthroughParser(list[float], list[float]),
        PassthroughParser(list[str], list[str]),
    ]

    parser = next((p for p in parsers if p.can_parse(for_parseable_type)), None)
    if parser is None:
        raise RuntimeError(
            f"The type '{for_parseable_type}' is not a supported parameter "
            f"type! If you'd like to use it, add a custom Parser for it and register it"
            " here in this function."
        )
    return parser
