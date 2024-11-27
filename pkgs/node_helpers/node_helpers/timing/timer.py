import logging
from collections import deque
from collections.abc import Callable
from functools import wraps
from timeit import default_timer as timer
from typing import Any, TypeVar, cast

FuncT = TypeVar("FuncT", bound=Callable[..., Any])


class Timer:
    """Use this module when timing any section or function of code.

    Example 1: As a context manager
    >>> timer = Timer(300)
    >>>
    >>> with timer:
    >>>     expensive_operation()
    >>>     other_operations()
    >>>
    >>>print(timer)

    Example 2: As a function decorator
    >>>timer = Timer(10)
    >>>
    >>>@timer
    >>>def cool_func():
    >>>     expensive_operation()
    >>>     other_operations()
    >>>
    >>>cool_func()
    >>>print(timer.fps)

    Example 3: Profiling multiple parts of a whole
    >>>timer = Timer(15)
    >>>
    >>>with timer:
    >>>     # do things
    >>>     with timer.child("SpecificThing"):
    >>>         # do a specific thing
    >>>
    >>># This report will show the whole time and the times of the child as percentage
    >>>print(timer)
    The timeout object will return false after the alloted time, ending the
    loop. This can be useful for tests.
    """

    _NO_SAMPLES_MSG = "No Samples!"
    _REPORT_INDENT = "\t"

    def __init__(self, samples: int | None = 1, name: str = "Timer", log: bool = False):
        """
        :param samples: The number of samples to extract the rolling average from
        :param name: The name of the timer, to put in reports
        :param log: Whether to log the timer report after it is finished
        """
        self.name = name
        self._log = log
        self._num_samples = samples
        self._samples: deque[float] = deque(maxlen=samples)
        self._sample_start: float | None = None
        self._children: dict[str, Timer] = {}

    def __call__(self, method: FuncT) -> FuncT:
        """This implements the decorator functionality of the timer"""

        @wraps(method)
        def on_call(*args: Any, **kwargs: Any) -> Any:
            with self:
                return method(*args, **kwargs)

        return cast(FuncT, on_call)

    def __repr__(self) -> str:
        return self.create_report()

    def __enter__(self) -> "Timer":
        self.begin()
        return self

    def __exit__(self, *args: object) -> None:
        self.end()
        if self._log:
            logging.info(f"{self}")

    @property
    def running(self) -> bool:
        """Return True if the timer is currently running"""
        return self._sample_start is not None

    def begin(self) -> None:
        """Start collection of a single timer sample"""
        if self.running:
            raise RuntimeError("The 'begin' method cannot be called twice in a row!")

        self._sample_start = timer()

    def end(self) -> None:
        """Finish collection of a single timer sample"""
        if not self.running:
            raise RuntimeError("'end' was called before 'begin'!")

        self._samples.append(self.current_elapsed)
        self._sample_start = None

    def create_report(self, depth: int = 0, parent_elapsed: float | None = None) -> str:
        """Create a report for this timer"""
        if len(self._samples):
            elapsed = self.elapsed
            if parent_elapsed is None:
                parent_elapsed = elapsed
            report = (
                f"{self.name.title()}("
                f"{round((elapsed / parent_elapsed) * 100, 2)}%, "
                f"elapsed={round(elapsed, 2)}, "
                f"fps={round(self.fps, 2)}, "
                f"samples={len(self._samples)})"
            )

            # Iterate over children appending their reports
            for child in self._children.values():
                report += "\n" + child.create_report(depth + 1, parent_elapsed=elapsed)
        else:
            # Prevent a ZeroDivisionError
            report = f"{self.name.title()}({self._NO_SAMPLES_MSG})"
        report = self._REPORT_INDENT * depth + report

        return report

    def child(self, name: str) -> "Timer":
        """Creates a child Timer of the given name, if none exists, and returns it."""
        if not self.running:
            raise RuntimeError(
                "You cannot create a child outside of the parent timers context!"
            )
        if name not in self._children:
            self._children[name] = Timer(samples=self._num_samples, name=name)
        return self._children[name]

    @property
    def elapsed(self) -> float:
        """Return the average elapsed time"""
        if len(self._samples) == 0:
            return 0.0
        return self.total_elapsed / len(self._samples)

    @property
    def total_elapsed(self) -> float:
        """Return the total time the timer spent active"""
        return float(sum(self._samples))

    @property
    def fps(self) -> float:
        """Return the average 'fps', or rather, 'operations per second'."""
        if len(self._samples) == 0:
            return 0.0
        return 1 / self.elapsed

    def reset(self) -> None:
        """Reset the timer, clearing all samples"""
        self._samples.clear()
        self._sample_start = None

    @property
    def current_elapsed(self) -> float:
        """Return the current elapsed time"""
        if not self.running:
            raise RuntimeError("You cannot collect a sample before starting the timer!")
        return timer() - self._sample_start  # type: ignore


class WarningTimer(Timer):
    """A variant of Timer that logs if the timer Hz is below a certain threshold"""

    def __init__(self, name: str, target_hz: float, samples: int | None = 1):
        super().__init__(samples=samples, name=name, log=False)
        self.target_hz = target_hz

    def __exit__(self, *args: object) -> None:
        super().__exit__(*args)
        if self.fps < self.target_hz:
            logging.warning(
                f"{self.name} is below target Hz of {self.target_hz} with "
                f"{self.fps:.2f} Hz"
            )
