import pytest
from node_helpers.timing import Timeout, Timer


def test_timeout_happy_path() -> None:
    """Test when a timer doesn't time out"""
    timeout = Timeout(seconds=999)

    assert timeout, "Non timed out timers should return True!"


def test_timeout_unhappy_path() -> None:
    """Test when a timer times out"""

    timeout = Timeout(seconds=0.015, raise_error=True)
    timer = Timer()
    with pytest.raises(TimeoutError), timer:
        while timeout:
            pass
    assert 0.02 > timer.elapsed > 0.01


def test_timeout_no_error() -> None:
    """Test when a timer times out, but an error should not be raised"""

    timeout = Timeout(seconds=0.015, raise_error=False)
    timer = Timer()
    with timer:
        while timeout:
            pass
    assert 0.02 > timer.elapsed > 0.01
    assert not timeout


def test_timeout_reset() -> None:
    """Test resetting with a different timeout"""

    timeout = Timeout(seconds=0.015)
    timer = Timer()
    with timer:
        while timeout:
            pass
    assert 0.02 > timer.elapsed > 0.01
    assert not timeout

    timeout.reset_seconds(0.01)

    timer = Timer()
    with timer:
        while timeout:
            pass
    assert not timeout
    assert 0.015 > timer.elapsed > 0.005
