from threading import Event

import pytest
from action_msgs.msg import GoalStatus
from node_helpers import futures, testing
from node_helpers.actions import (
    ActionElement,
    ActionGroup,
    ActionSequence,
    AlreadyRunningActionsError,
    NoRunningActionsError,
    ParallelActionSequences,
)
from node_helpers.robust_rpc import RobustRPCException
from node_helpers.timing import TestingTimeout
from node_helpers_msgs.action import RobustActionExample

from .server.test_base_handler_and_worker import (
    ExampleActionClient,
    ExampleActionServer,
    example_action_client,  # noqa: F401
    example_action_server,  # noqa: F401
)
from .server.utils import CoolError


def test_action_sequence_basic_usage_without_feedback(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test the high-level ActionSequence, ActionGroup functionality, with actions that
    don't use 'continue' feedback triggers"""

    group_a = ActionGroup(
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
    )
    group_b = ActionGroup(
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
    )
    sequence = ActionSequence(group_a, group_b)

    results = sequence.execute()

    # Verify actions were run and results match up with the number of actions
    assert len(results) == 2  # two groups
    assert len(results[0]) == 2  # two actions, group 1
    assert len(results[1]) == 1  # one action, group 2
    assert len(group_a._running_actions) == 0
    assert len(group_b._running_actions) == 0


def test_action_group_send_goals(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Basic test that send_goals sends goals and waits for results to finish"""
    group = ActionGroup(
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
    )
    results = group.send_goals()
    assert len(results) == 3

    # The running futures and action triggers should be reset
    assert len(group._running_actions) == 0


def test_cancel_ongoing_goals(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Try cancelling goals that are still running"""

    group = ActionGroup(
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
    )

    # It should be okay to cancel when there are no running actions. This is helpful
    # because when ActionWorkers are handling their on_cancel's, they can just cancel
    # any action groups they have (regardless of whether they were doing anything).
    assert len(group._running_actions) == 0
    group.cancel_goals()  # No exception

    # Now run the action, ensuring it never finishes by adding a done_event
    example_action_server.handler.done_event = Event()
    group.send_goals_async()
    ongoing_actions = group._running_actions
    assert len(ongoing_actions) == 3

    # Now call cancel_goals and validate all actions ended in cancellation
    group.cancel_goals()

    # The group should clear internal running actions after cancellation
    assert len(group._running_actions) == 0

    # All result futures should be completed
    assert all(a.result_future.done() for a in ongoing_actions)

    # All clients should report cancelled
    assert all(
        a.goal_handle.status == GoalStatus.STATUS_CANCELED for a in ongoing_actions
    )


def test_cancel_goals_with_finished_futures(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test you can still safely call 'cancel_goals' if the action finished already"""
    group = ActionGroup(
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
    )
    group.send_goals_async()

    # Wait until the underlying actions have finished
    futures.wait_for_futures([a.result_future for a in group._running_actions], object)
    assert len(group._running_actions) == 2
    assert all(a.result_future.done() for a in group._running_actions)

    # This shouldn't fail
    group.cancel_goals()
    assert len(group._running_actions) == 0


def test_action_group_accepts_iterables_and_elements(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """A simple test that you can put generators in an ActionGroup.

    For example, the following should be acceptable:
    ActionGroup(ActionElement(client=client, ...) for client in clients)

    You should also be able to:
    ActionGroup(e for e in elements, element_c, element_d)
    """
    element_a = ActionElement(example_action_client.client, RobustActionExample.Goal())
    element_b = ActionElement(example_action_client.client, RobustActionExample.Goal())
    element_c = ActionElement(example_action_client.client, RobustActionExample.Goal())

    as_list = [element_a, element_b, element_c]
    as_generator = (e for e in as_list)
    as_mix_of_generator_and_items = ((e for e in (element_a, element_b)), element_c)

    assert ActionGroup(as_list)._action_elements == as_list
    assert ActionGroup(*as_list)._action_elements == as_list
    assert ActionGroup(as_generator)._action_elements == as_list
    assert ActionGroup(*as_mix_of_generator_and_items)._action_elements == as_list


def test_action_group_basic_usage_without_feedback(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test the high-level ActionGroup functionality, with actions that don't use
    'continue' feedback triggers.

    It also validates that errors are raised when 'wait_for_results' is called.
    """

    example_action_server.handler.done_event = Event()

    group = ActionGroup(
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
    )
    assert len(group._running_actions) == 0

    group.send_goals_async()
    assert len(group._running_actions) == 2
    assert all(not r.result_future.done() for r in group._running_actions)
    assert all(not r.next_action_trigger.is_set() for r in group._running_actions)

    # Allow the action to finish
    example_action_server.handler.done_event.set()
    group.wait_for_feedback_triggers()
    assert len(group._running_actions) == 2
    assert all(r.next_action_trigger.is_set() for r in group._running_actions)
    assert all(r.result_future.done() for r in group._running_actions)

    results = group.wait_for_results()
    assert len(results) == 2
    assert len(group._running_actions) == 0


def test_action_group_basic_usage_with_feedback(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test the high-level ActionGroup functionality, with an action group that has the
    feedback_callback specified
    """

    example_action_server.handler.done_event = Event()

    # Add a feedback callback that immediately sets the feedback_trigger
    def feedback_callback(feedback: RobustActionExample.Feedback) -> bool:
        assert isinstance(feedback, RobustActionExample.Feedback)
        return True

    group = ActionGroup(
        ActionElement(
            example_action_client.client, RobustActionExample.Goal(), feedback_callback
        ),
        ActionElement(
            example_action_client.client, RobustActionExample.Goal(), feedback_callback
        ),
    )
    assert len(group._running_actions) == 0

    group.send_goals_async()
    group.wait_for_feedback_triggers()

    # None of the actions should have their results set yet, but all feedback triggers
    # should be set by now
    assert len(group._running_actions) == 2
    assert all(r.next_action_trigger.is_set() for r in group._running_actions)
    assert all(not r.result_future.done() for r in group._running_actions)

    # All of the events must be unique objects
    assert len({r.next_action_trigger for r in group._running_actions}) == 2

    # Allow the action to finish
    example_action_server.handler.done_event.set()

    results = group.wait_for_results()
    assert all(isinstance(r, RobustActionExample.Result) for r in results), (
        "The wait_for_results method should return the underlying result objects, not "
        "the *_GetResult_Response message!"
    )
    assert len(group._running_actions) == 0
    assert len(results) == 2


def test_action_group_exception_cases(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test all the usages of ActionGroup that should end in exceptions"""

    example_action_server.handler.raise_exception = True

    with pytest.raises(ValueError):
        # You shouldn't be able to create an ActionGroup without ActionElements
        ActionGroup()

    group = ActionGroup(
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
    )

    # Waiting for triggers or results should fail if the actions haven't been started
    with pytest.raises(NoRunningActionsError):
        group.wait_for_feedback_triggers()

    with pytest.raises(NoRunningActionsError):
        group.wait_for_results()

    with pytest.raises(NoRunningActionsError):
        next(group.yield_for_results(yield_interval=999))

    group.send_goals_async()
    with pytest.raises(AlreadyRunningActionsError):
        # Running send_goals_async twice should fail
        group.send_goals_async()

    # Validate exceptions are raised in wait_for_feedback_triggers
    with pytest.raises(RobustRPCException.like(CoolError)):
        group.wait_for_feedback_triggers()

    # Validate exceptions are raised in yield_for_feedback_triggers
    with pytest.raises(RobustRPCException.like(CoolError)):
        testing.exhaust_generator(group.yield_for_feedback_triggers())

    # Validate exceptions are raised in yield_for_results
    with pytest.raises(RobustRPCException.like(CoolError)):
        testing.exhaust_generator(group.yield_for_results(yield_interval=0))

    # Validate exceptions are raised in wait_for_results
    with pytest.raises(RobustRPCException.like(CoolError)):
        group.wait_for_results()


def test_action_group_yield_for_results(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    # Set a 'done_event' to ensure the actions don't finish until my say so
    example_action_server.handler.done_event = Event()
    group = ActionGroup(
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ActionElement(example_action_client.client, RobustActionExample.Goal()),
    )
    group.send_goals_async()

    # Normally this would be called like "result = yield from group.yield_for_results()"
    # but since we aren't running in an actionworker 'run' loop, we call next manually
    yield_generator = group.yield_for_results(yield_interval=0)

    # Check that it yields many times over before a result is given
    for _ in range(100):
        assert next(yield_generator) is None

    # Validate none of the futures are done yet
    assert all(not a.result_future.done() for a in group._running_actions)

    # Now allow all the actions to continue, and continue yielding meanwhile
    example_action_server.handler.done_event.set()
    with pytest.raises(StopIteration) as error:
        while True:
            assert next(yield_generator) is None

    assert error.value.value is not None
    assert len(group._running_actions) == 0, "All actions should have been cleared!"


def test_action_sequence_exception_cases(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    example_action_server.handler.raise_exception = True
    sequence = ActionSequence(
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal())
            for _ in range(3)
        ),
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
    )

    # Test that execute raises an exception
    with pytest.raises(RobustRPCException.like(CoolError)):
        sequence.execute()

    # Test that yield_for_execution raises an exception
    with pytest.raises(RobustRPCException.like(CoolError)):
        testing.exhaust_generator(sequence.yield_for_execution())

    # The first group should have 3 running actions, the second group SHOULD NOT HAVE
    # HAD ANY ACTIONS RUN! This basically validates that the exception was caught,
    # raised, and the sequence was able to stop the second group from running.
    assert len(sequence._action_groups[0]._running_actions) == 3
    assert len(sequence._action_groups[1]._running_actions) == 0

    # Test that wait_for_results raises an exception
    with pytest.raises(RobustRPCException.like(CoolError)):
        sequence.wait_for_results()


def test_parallel_action_sequences_basic_execution(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test basic parallel execution of multiple action sequences."""

    # Create two action sequences
    sequence1 = ActionSequence(
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
    )

    sequence2 = ActionSequence(
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
    )

    # Create ParallelActionSequences with the two sequences
    parallel_sequences = ParallelActionSequences(sequence1, sequence2)

    # Execute the sequences in parallel
    _, results = testing.exhaust_generator(parallel_sequences.yield_for_execution())

    # Verify that results are returned for both sequences
    assert len(results) == 2  # Two sequences
    assert len(results[0]) == 2  # Two action groups in sequence1
    assert len(results[1]) == 2  # Two action groups in sequence2

    # Verify that each action group has one result
    assert all(len(group_results) == 1 for group_results in results[0])
    assert all(len(group_results) == 1 for group_results in results[1])


def test_parallel_action_sequences_with_feedback(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test parallel execution of action sequences with feedback callbacks."""

    # Event to control when feedback is received
    feedback_event = Event()

    # Feedback callback that waits for the event to be set
    def feedback_callback(feedback: RobustActionExample.Feedback) -> bool:
        return feedback_event.is_set()

    # Set up sequences with feedback callbacks
    sequence1 = ActionSequence(
        ActionGroup(
            ActionElement(
                example_action_client.client,
                RobustActionExample.Goal(),
                feedback_callback=feedback_callback,
            ),
        ),
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
    )

    sequence2 = ActionSequence(
        ActionGroup(
            ActionElement(
                example_action_client.client,
                RobustActionExample.Goal(),
                feedback_callback=feedback_callback,
            ),
        ),
    )

    # Start the parallel sequences
    parallel_sequences = ParallelActionSequences(sequence1, sequence2)

    # Since we're simulating, we'll step through the execution manually
    sequence_generators = [
        seq.yield_for_execution() for seq in parallel_sequences._action_sequences
    ]

    # Initially, feedback callbacks will block waiting for feedback_event
    # Let's step through a few iterations
    for _ in range(100):
        for gen in sequence_generators:
            assert next(gen) is None

    # At this point, both sequences should be waiting at the feedback trigger
    # Set the feedback event to allow them to proceed
    feedback_event.set()

    # Continue stepping through execution
    result_count = 0
    timeout = TestingTimeout(10)
    for sequence_idx, gen in enumerate(sequence_generators):
        while timeout:
            try:
                next(gen)

            except StopIteration as finished:
                result_count += 1
                assert finished.value is not None  # noqa: PT017
                if sequence_idx == 0:
                    assert len(finished.value) == 3  # noqa: PT017
                else:
                    assert len(finished.value) == 1  # noqa: PT017
                break

    assert result_count == 2  # 2 action groups total


def test_parallel_action_sequences_exception_cases(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test that exceptions in one sequence do not affect others."""

    # Configure the server to raise an exception when processing goals
    example_action_server.handler.raise_exception = True

    # Set up sequences
    sequence1 = ActionSequence(
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
    )

    sequence2 = ActionSequence(
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
    )

    # Start the parallel sequences
    parallel_sequences = ParallelActionSequences(sequence1, sequence2)

    # Execute and expect an exception
    with pytest.raises(RobustRPCException.like(CoolError)):
        testing.exhaust_generator(parallel_sequences.yield_for_execution())

    # Verify that all sequences ended, essentially verifying that the exception in
    # parallel sequences is caught, stored, and re-raised once the system has fully
    # synchronized
    for sequence in parallel_sequences._action_sequences:
        for action_group in sequence._action_groups:
            for running_action in action_group._running_actions:
                assert running_action.result_future.done()


def test_parallel_action_sequences_cancellation(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test that ParallelActionSequences can be canceled properly."""

    # Create an event to control when the actions finish
    example_action_server.handler.done_event = Event()

    # Create two action sequences
    sequence1 = ActionSequence(
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
    )

    sequence2 = ActionSequence(
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
            ActionElement(example_action_client.client, RobustActionExample.Goal()),
        ),
    )

    # Create ParallelActionSequences with the two sequences
    parallel_sequences = ParallelActionSequences(sequence1, sequence2)

    # Edit the action server with a separate done event, so it never finishes
    example_action_server.handler.done_event = Event()

    # Start the parallel sequences
    execute_generator = parallel_sequences.yield_for_execution()
    next(execute_generator)

    # Verify the underlying actions are running, and the correct ones at that
    assert len(sequence1._action_groups[0]._running_actions) == 2
    assert all(
        not a.result_future.done() for a in sequence1._action_groups[0]._running_actions
    )
    assert len(sequence1._action_groups[1]._running_actions) == 0

    assert len(sequence2._action_groups[0]._running_actions) == 1
    assert all(
        not a.result_future.done() for a in sequence2._action_groups[0]._running_actions
    )
    assert len(sequence2._action_groups[1]._running_actions) == 0

    # Cancel goals
    parallel_sequences.cancel()

    # Verify that all actions are canceled
    for sequence in parallel_sequences._action_sequences:
        for action_group in sequence._action_groups:
            assert len(action_group._running_actions) == 0


def test_parallel_action_sequences_exception_prevents_other_actions(
    example_action_server: ExampleActionServer,  # noqa: F811
    example_action_client: ExampleActionClient,  # noqa: F811
) -> None:
    """Test that exceptions prevent other actions from being called."""

    # Configure the server to raise an exception when processing goals
    example_action_server.handler.raise_exception = True

    # Set up sequences
    sequence1 = ActionSequence(
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal())
            for _ in range(2)
        ),
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal())
            for _ in range(3)
        ),
    )

    sequence2 = ActionSequence(
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal())
            for _ in range(4)
        ),
        ActionGroup(
            ActionElement(example_action_client.client, RobustActionExample.Goal())
            for _ in range(5)
        ),
    )

    # Start the parallel sequences
    parallel_sequences = ParallelActionSequences(sequence1, sequence2)

    # Execute and expect an exception
    with pytest.raises(RobustRPCException.like(CoolError)):
        testing.exhaust_generator(parallel_sequences.yield_for_execution())

    # Verify that no further actions were called after the exception
    assert len(sequence1._action_groups[0]._running_actions) == 2
    assert len(sequence1._action_groups[1]._running_actions) == 0
    assert len(sequence2._action_groups[0]._running_actions) == 4
    assert len(sequence2._action_groups[1]._running_actions) == 0
