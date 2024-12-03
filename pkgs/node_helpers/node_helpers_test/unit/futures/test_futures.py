from threading import Event

import pytest
from node_helpers import futures
from node_helpers.testing import DynamicContextThread
from rclpy import Future


def test_wait_for_futures_waits_on_all_even_after_exception() -> None:
    """Ensure that wait_for_futures will always wait for _all_ futures to complete
    before raising the first exception that occured"""

    all_futures = [Future(), Future(), Future(), Future()]
    finished = Event()

    def wait_on_futures() -> None:
        try:
            futures.wait_for_futures(all_futures, object)
        except ValueError:
            finished.set()

    with DynamicContextThread(wait_on_futures):
        assert not finished.is_set()

        # Set the first two exceptions
        all_futures[0].set_result(None)
        all_futures[1].set_result(None)

        # Have the third end in an exception
        all_futures[2].set_exception(ValueError)

        assert not finished.wait(0.1)

        # Finally, let the function continue
        all_futures[3].set_result(None)

        assert finished.wait(0.1)


def test_yield_for_future_on_unfinished_future() -> None:
    """Test yield_for_futures basic usage"""
    future = Future()
    expected_result = 3

    generator = futures.yield_for_future(future, object, yield_interval=0)
    for _ in range(3):
        assert next(generator) is None

    # Now set the future, and ensure it returns the expected result
    future.set_result(expected_result)
    with pytest.raises(StopIteration) as error:
        next(generator)

    assert error.value.value == expected_result


def test_yield_for_future_on_finished_future() -> None:
    future = Future()
    expected_value = 20
    future.set_result(expected_value)

    generator = futures.yield_for_future(future, object, yield_interval=0)

    # This should raise a StopIteration immediately, since the future is done
    with pytest.raises(StopIteration) as error:
        next(generator)
    assert error.value.value == expected_value
