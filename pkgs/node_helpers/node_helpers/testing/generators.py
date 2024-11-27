from collections.abc import Generator
from typing import Any, TypeVar

RET_VAL = TypeVar("RET_VAL")
YIELD_VAL = TypeVar("YIELD_VAL")
INPUT_VAL = TypeVar("INPUT_VAL")


def exhaust_generator(
    generator: Generator[YIELD_VAL, Any, RET_VAL],
) -> tuple[tuple[YIELD_VAL, ...], RET_VAL]:
    """Exhaust a generator and return the yielded values and the return value"""
    yielded: list[YIELD_VAL] = []

    try:
        while True:
            yielded.append(next(generator))
    except StopIteration as e:
        returned = e.value

    return tuple(yielded), returned
