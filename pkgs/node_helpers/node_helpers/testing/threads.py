import re
import threading
from abc import ABC, abstractmethod
from collections.abc import Callable
from functools import partial
from types import TracebackType
from typing import Any, TypeVar

T = TypeVar("T", bound="ContextThread")


class ContextThread(ABC):
    """Manages the lifetime of a thread using a context manager"""

    def __init__(self, name: str) -> None:
        self.running = False
        self.exception: Exception | None = None
        """If an exception is raised, it'll be re-raised upon exiting the context"""

        self._thread = threading.Thread(
            name=name,
            target=self._record_exceptions,
            daemon=True,
        )

    @abstractmethod
    def run(self) -> None:
        """The behavior to run in the thread"""

    def _record_exceptions(self) -> None:
        try:
            self.run()
        except Exception as e:  # noqa: BLE001
            self.exception = e

    def __enter__(self: T) -> T:
        self.running = True
        self.exception = None
        self._thread.start()
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_val: BaseException | None,
        _exc_tb: TracebackType | None,
    ) -> None:
        self.running = False
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise RuntimeError(f"Thread {self._thread.name} did not stop")

        if self.exception:
            raise self.exception


class DynamicContextThread(ContextThread):
    """A ContextThread that lets you pass a function and arguments on initialization

    Mirrors the Thread(...) initialization API
    """

    def __init__(self, target: Callable[..., Any], *args: Any, **kwargs: Any):
        super().__init__(name=f"{target.__name__} DynamicContextThread")
        self._target = partial(target, *args, **kwargs)

    def run(self) -> None:
        self._target()


def get_unclosed_threads(allowable_threads: list[str] | None = None) -> list[str]:
    """A convenient function that returns threads that shouldn't be alive inbetween
    tests.

    :param allowable_threads: A list of regular expressions that match to
        thread names that are allowed to stay live inbetween tests
    :returns: A list of thread names that didn't match the allowable thread regexes
    """
    if allowable_threads is None:
        allowable_threads = []
    allowable_threads += [
        r"pydevd\.Writer",
        r"pydevd\.Reader",
        r"pydevd\.CommandThread",
        r"profiler\.Reader",
        r"MainThread",
    ]

    open_threads = []

    for thread in threading.enumerate():
        matched = False
        for name_pattern in allowable_threads:
            if re.match(name_pattern, thread.name):
                matched = True
                break

        if not matched:
            open_threads.append(thread)

    return [o.name for o in open_threads]
