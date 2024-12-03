from collections.abc import Generator
from time import sleep, time


def generator_sleep(seconds: float) -> Generator[None, None, None]:
    """Like time.sleep, but routinely yields to allow for preemption"""

    start = time()
    elapsed = 0.0

    while elapsed < seconds:
        elapsed = time() - start

        if 0 <= seconds - elapsed < _SLEEP_INTERVAL:
            # The sleep is too short to bother with preemption
            sleep(seconds - elapsed)

            # Yield at least once, in case the sleep was too short and the function
            # hasn't yielded yet.
            yield None
            return

        sleep(_SLEEP_INTERVAL)
        yield None


def yield_until(delay: float) -> Generator[None, None, None]:
    """Yields until the provided delay has passed without sleeping"""

    start = time()
    while time() - delay < start:
        yield None


_SLEEP_INTERVAL = 0.005
