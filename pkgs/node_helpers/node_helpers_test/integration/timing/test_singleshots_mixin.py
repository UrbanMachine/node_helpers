import gc
from collections.abc import Generator
from queue import Empty, Queue
from time import sleep
from typing import Any, Literal

import pytest
from node_helpers.testing import set_up_node
from node_helpers.timing import SingleShotMixin, Timeout
from rclpy.exceptions import InvalidHandle
from rclpy.node import Node


class SingleShotNode(Node, SingleShotMixin):
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__("single_shot_node", *args, **kwargs)


@pytest.fixture(params=[False, True])
def single_shot_node(
    request: pytest.FixtureRequest,
) -> Generator[SingleShotNode, None, None]:
    """Yields two versions of the single shot node: multi-threaded, single-threaded"""
    yield from set_up_node(
        node_class=SingleShotNode,
        namespace="lol_idk",
        node_name="single_shot_node",
        parameter_overrides=[],
        multi_threaded=request.param,
    )


def test_single_shots_run_once(single_shot_node: SingleShotNode) -> None:
    """Basic test that when a single shot is created, yields only one response"""
    results: Queue[Literal[1]] = Queue()
    single_shot_node.create_single_shot_timer(
        timer_period_sec=0,
        callback=lambda: results.put(1),
    )

    assert results.get(timeout=0.2) == 1
    with pytest.raises(Empty):
        results.get(timeout=0.2)


def test_single_shots_return_in_order(single_shot_node: SingleShotNode) -> None:
    """This verifies single shots are run 'in order' of their time_period_sec by
    creating multiple single shots incrementally going up in time, then verifying the
    order returned correctly"""

    results: Queue[int] = Queue()
    n_single_shots = 10
    time_offset = 0.3

    # Create several timers in reverse order to make sure only the time_period_sec is
    # the only thing affecting order
    for i in range(n_single_shots - 1, -1, -1):
        single_shot_node.create_single_shot_timer(
            timer_period_sec=i * time_offset,
            callback=lambda i=i: results.put(i),  # type: ignore
        )

    # Now wait for all the single shots and validate they returned in order
    for i in range(n_single_shots):
        assert results.get() == i

    # There should not be anything else
    with pytest.raises(Empty):
        results.get(timeout=0.1)


def test_timers_cancelled_and_dereferenced(single_shot_node: SingleShotNode) -> None:
    """Test that creating many single shots works fine and robustly, and ends up without
    more references than we began with"""

    results: Queue[None] = Queue(maxsize=1)
    n_single_shots = 1000

    timers = tuple(
        single_shot_node.create_single_shot_timer(
            timer_period_sec=0,
            callback=lambda: results.put(None, timeout=30),
        )
        for _ in range(n_single_shots)
    )

    # Get the number of referrants to the timer objects before allowing their callbacks
    # to complete
    referrers_count_before = len(gc.get_referrers(*timers))
    task_count_before = len(single_shot_node.executor._tasks)
    assert task_count_before > 0, "Some tasks should be added for timers to work"

    # Now wait for all the single shots, allowing the callbacks to finish (because the
    # queue has a maxsize=1, it will block the timer callbacks until get() is called)
    for _ in range(n_single_shots):
        results.get(timeout=30)

    # Validate the timer references have dropped by n_single_shots and no more tasks
    # exist in the executor. This can take a moment when running with a multithreaded
    # executor, so we run this with a timeout.
    timeout = Timeout(seconds=10)

    def ros_has_dropped_references() -> bool:
        current_n_referrers = len(gc.get_referrers(*timers))
        return referrers_count_before - current_n_referrers >= n_single_shots - 1

    while not ros_has_dropped_references() and timeout:
        # Garbage collect so that `__del__` methods in the timer get triggered
        gc.collect()
        # Wake the executor to let it know that something has changed
        single_shot_node.executor.wake()
        sleep(0.1)

    task_count_after = len(single_shot_node.executor._tasks)
    assert results.qsize() == 0
    assert task_count_after <= 1, "Only the 'wake()' task (or no tasks) should be left!"

    # Validate all timers were destroyed
    for timer in timers:
        with pytest.raises(InvalidHandle):
            timer.is_ready()


def test_timers_creation_allowed_before_executor() -> None:
    """Test that you can safely call create_single_shot_timer in a node __init__ and it
    will call as soon as the node is spun up with an executor"""

    requests: Queue[None] = Queue()

    class SingleShotOnInitNode(Node, SingleShotMixin):
        def __init__(self, *args: Any, **kwargs: Any):
            super().__init__("single_shot_node", *args, **kwargs)
            self.create_single_shot_timer(
                timer_period_sec=0, callback=lambda: requests.put(None)
            )

    # The following will run the node __init__ then assign it an executor. This should
    # work just fine.
    node_generator = set_up_node(SingleShotOnInitNode, "node_namespace", "node_name")
    next(node_generator)

    # Ensure a request is received
    requests.get(timeout=15)

    # Shut down the node
    tuple(node_generator)
