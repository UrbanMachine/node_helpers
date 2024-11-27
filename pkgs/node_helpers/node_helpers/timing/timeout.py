from timeit import default_timer as timer


def has_timed_out(start: float, timeout: float) -> bool:
    """Convenience method to return true if a period of time has elapsed
    :param start: The start time of some arbitrary event
    :param timeout: The time in seconds to check against
    :return: True if the period has elapsed
    """
    return timer() - start > timeout


class Timeout:
    """This class is useful for checking if a time limit has finished.
    It is useful for tests, or writing blocking calls that have the option to raise
    TimeoutErrors.
    Optionally raise a TimeoutError if the time limit has been reached.

    >>>timeout = Timeout(seconds=3)
    >>>while timeout and other_condition():
    >>>     pass

    If `other_condition()` never returns false, then the timeout object returns false
    and optionally raises a `TimeoutError` after 3 seconds.
    """

    TIMEOUT_MESSAGE = "The Timeout of {seconds} seconds has been reached!"

    def __init__(
        self,
        seconds: float,
        raise_error: bool = False,
        timeout_message: str = TIMEOUT_MESSAGE,
    ):
        self.seconds = seconds
        self._start = timer()
        self._raise_error = raise_error
        self._timeout_message = timeout_message

    def __bool__(self) -> bool:
        if has_timed_out(self._start, self.seconds):
            if self._raise_error:
                raise TimeoutError(self._timeout_message.format(seconds=self.seconds))
            else:
                return False
        return True

    def reset(self) -> None:
        """Reset the timer"""
        self._start = timer()

    def reset_seconds(self, seconds: float) -> None:
        """Reset the timer and set a new time limit.
        This is useful for reusing the same object for changing time limits.
        :param seconds: The new time limit"""
        self.seconds = seconds
        self.reset()


class TestingTimeout(Timeout):
    """This class is useful for raising a TimeoutError once a time limit has finished.
    It will always raise a TimeoutError, and is mainly used for tests"""

    def __init__(self, seconds: float, timeout_message: str = Timeout.TIMEOUT_MESSAGE):
        super().__init__(seconds, raise_error=True, timeout_message=timeout_message)
