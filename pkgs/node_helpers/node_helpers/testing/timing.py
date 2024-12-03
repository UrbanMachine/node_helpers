from node_helpers.timing import Timeout

"""Handy mock for Timeout that is always expired.
Helpful for testing modules that sleep or wait for timeouts."""


class MockTimeout(Timeout):
    def __init__(
        self,
        seconds: float,
        raise_error: bool = False,
        timeout_message: str = "Timeout",
        default_state: bool = False,  # Always expired by default
    ) -> None:
        self.seconds = seconds
        self.mock_timeout_state = default_state

    def reset_seconds(self, seconds: float) -> None:
        self.seconds = seconds

    def __bool__(self) -> bool:
        return self.mock_timeout_state

    def set_active(self, active: bool) -> None:
        self.mock_timeout_state = active
