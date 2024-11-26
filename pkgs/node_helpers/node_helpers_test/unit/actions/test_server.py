from collections.abc import Generator
from dataclasses import dataclass
from unittest.mock import Mock

import pytest
from node_helpers.actions.server import (
    ActionTimeoutError,
    ActionWorker,
    NoResultSetError,
)
from rclpy.action.server import ServerGoalHandle


class ExampleGoal:
    pass


class ExampleFeedback:
    pass


@dataclass
class ExampleResult:
    value: str
    error_name: str = ""
    error_description: str = ""


ExampleWorkerType = ActionWorker[ExampleGoal, ExampleFeedback, ExampleResult]


class ExampleException(Exception):
    pass


class ExampleWorker(ExampleWorkerType):
    """A basic action that produces feedback, responds to errors and cancellations, and
    provides a result
    """

    FEEDBACK_COUNT = 3

    def __init__(self, goal_handle: ServerGoalHandle):
        super().__init__(goal_handle)

        self.canceled = False  # This says a lot about our society

    def run(self) -> Generator[ExampleFeedback | None, None, None]:
        for _ in range(self.FEEDBACK_COUNT):
            yield ExampleFeedback()
        self.result = ExampleResult("complete")

    def on_cancel(self) -> ExampleResult:
        self.canceled = True
        return ExampleResult("cancel")


def test_action_worker_normal_operation() -> None:
    """Tests that a successful worker has its feedback and results published"""

    goal_handle = Mock()
    goal_handle.is_cancel_requested = False

    worker = ExampleWorker(goal_handle)
    result = worker.execute_callback(None)

    _assert_complete(worker, goal_handle, result)
    assert goal_handle.publish_feedback.call_count == ExampleWorker.FEEDBACK_COUNT


def test_action_worker_canceled() -> None:
    """Tests that a worker can be canceled, and that the worker's cancel callback is
    called
    """

    goal_handle = Mock()
    goal_handle.is_cancel_requested = True

    worker = ExampleWorker(goal_handle)
    result = worker.execute_callback(None)

    _assert_canceled(worker, goal_handle, result)


class NoResultWorker(ExampleWorker):
    """A worker implementation that forgot to set the result value after execution"""

    def run(self) -> Generator[ExampleFeedback | None, None, None]:
        for _ in range(self.FEEDBACK_COUNT):
            yield ExampleFeedback()


def test_action_worker_no_result() -> None:
    """Tests that the worker errors out if subclasses fail to set the result attribute
    after successful execution
    """
    goal_handle = Mock()
    goal_handle.is_cancel_requested = False

    worker = NoResultWorker(goal_handle)

    with pytest.raises(NoResultSetError):
        worker.execute_callback(None)


class ForeverWorker(ExampleWorker):
    """A worker implementation that will run forever unless stopped by a timeout"""

    def run(self) -> Generator[ExampleFeedback | None, None, None]:
        while True:
            yield ExampleFeedback()


def test_action_worker_timeout() -> None:
    """Tests that long-running workers can be interrupted by a timeout"""

    goal_handle = Mock()
    goal_handle.is_cancel_requested = False

    worker = ForeverWorker(goal_handle)

    with pytest.raises(ActionTimeoutError):
        worker.execute_callback(1.0)


class NoneYieldingWorker(ExampleWorker):
    """A worker that yields None instead of a feedback object"""

    def run(self) -> Generator[ExampleFeedback | None, None, None]:
        for _ in range(self.FEEDBACK_COUNT):
            yield None
        self.result = ExampleResult("complete")


def test_action_worker_none_yielding() -> None:
    """Tests that workers are allowed to yield None, and no feedback is provided when
    they do
    """
    goal_handle = Mock()
    goal_handle.is_cancel_requested = False

    worker = NoneYieldingWorker(goal_handle)
    result = worker.execute_callback(None)

    _assert_complete(worker, goal_handle, result)
    goal_handle.publish_feedback.assert_not_called()


def _assert_complete(
    worker: ExampleWorkerType, goal_handle: Mock, result: ExampleResult
) -> None:
    assert isinstance(result, ExampleResult)
    assert result.value == "complete"
    goal_handle.succeed.assert_called_once()
    assert worker.done


def _assert_canceled(
    worker: ExampleWorkerType, goal_handle: Mock, result: ExampleResult
) -> None:
    assert isinstance(result, ExampleResult)
    assert result.value == "cancel"
    goal_handle.canceled.assert_called_once()
    assert worker.done
