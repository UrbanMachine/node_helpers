import logging
import traceback
from types import TracebackType


class ExceptionCollector:
    """A helper class for collecting exceptions in cases where multiple operations must
    finish before an exception can be re-raised.

    In the future, a GroupException may be implemented using python 3.12 features.
    Currently, maybe_raise() raises the first collected exception.
    """

    def __init__(self) -> None:
        self.exceptions: list[tuple[BaseException, TracebackType | None]] = []

    def __enter__(self) -> None:
        pass

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_val: BaseException | None,
        _exc_tb: TracebackType | None,
    ) -> bool:
        if _exc_val:
            self.exceptions.append((_exc_val, _exc_tb))
        return True  # Suppress the exception

    def had_exceptions(self) -> bool:
        return bool(self.exceptions)

    def maybe_raise(self, log: bool = True) -> None:
        if self.exceptions:
            if log:
                for exc, tb in self.exceptions:
                    logging.error(f"Exception occurred: {exc}")
                    logging.error("Traceback:\n" + "".join(traceback.format_tb(tb)))
            raise self.exceptions[0][0]
