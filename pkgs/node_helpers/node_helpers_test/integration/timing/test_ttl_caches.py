from typing import cast
from unittest.mock import Mock

from node_helpers.timing import ttl_cached


def test_basic_use_on_bound_methods() -> None:
    ttl = 0.25

    class SomeClass:
        return_value = 3.0

        @ttl_cached(seconds=ttl)
        def some_fn(self) -> float:
            return self.return_value

    some_instance = SomeClass()
    timer = Mock()
    some_instance.some_fn.cache._get_time = timer

    timer.return_value = 0
    assert some_instance.some_fn() == 3.0
    assert some_instance.some_fn() == 3.0
    some_instance.return_value = 4.0
    assert some_instance.some_fn() == 3.0
    assert some_instance.some_fn() == 3.0
    timer.return_value = ttl + 0.1
    assert some_instance.some_fn() == 4.0

    # Also try busy looping continuously while waiting for a value to change
    some_instance.return_value = 5.0
    assert some_instance.some_fn() == 4.0
    while some_instance.some_fn() == 4.0:
        timer.return_value += 0.1
    assert some_instance.some_fn() == 5.0


def test_basic_use_on_unbound_methods() -> None:
    ttl = 0.25

    value_holder = Mock()
    value_holder.return_value = 3.0

    @ttl_cached(seconds=ttl)
    def some_fn() -> float:
        return cast(float, value_holder())

    timer = Mock()
    some_fn.cache._get_time = timer
    timer.return_value = 0

    assert some_fn() == 3.0
    assert some_fn() == 3.0
    value_holder.return_value = 4.0
    assert some_fn() == 3.0
    assert some_fn() == 3.0
    timer.return_value = ttl + 0.1
    assert some_fn() == 4.0

    # Also try busy looping continuously while waiting for a value to change
    value_holder.return_value = 5.0
    assert some_fn() == 4.0
    while some_fn() == 4.0:
        timer.return_value += 0.1
    assert some_fn() == 5.0


def test_not_called_even_when_returns_none() -> None:
    """Test that the TTL functionality respects the None return value"""
    ttl = 0.25
    underlying_mock = Mock()

    class SomeClass:
        @ttl_cached(seconds=ttl)
        def some_fn(self) -> None:
            underlying_mock()

    some_instance = SomeClass()

    timer = Mock()
    some_instance.some_fn.cache._get_time = timer
    timer.return_value = 0

    assert underlying_mock.call_count == 0
    some_instance.some_fn()
    assert underlying_mock.call_count == 1
    some_instance.some_fn()
    assert underlying_mock.call_count == 1
    timer.return_value = ttl + 0.1
    some_instance.some_fn()
    assert underlying_mock.call_count == 2
    some_instance.some_fn()
    assert underlying_mock.call_count == 2


def test_ttl_editing() -> None:
    """Test editing the ttl property on a method"""

    class SomeClass:
        return_value = 3.0

        @ttl_cached(seconds=999.0)
        def some_fn(self) -> float:
            return self.return_value

    some_instance = SomeClass()

    assert some_instance.some_fn() == 3.0
    assert some_instance.some_fn() == 3.0
    some_instance.return_value = 4.0
    assert some_instance.some_fn() == 3.0
    assert some_instance.some_fn() == 3.0
    some_instance.some_fn.cache.ttl_seconds = 0.0
    assert some_instance.some_fn() == 4.0
