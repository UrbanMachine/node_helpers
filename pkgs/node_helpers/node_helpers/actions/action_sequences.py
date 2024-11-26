import itertools
from collections.abc import Callable, Generator, Iterable
from dataclasses import dataclass
from threading import Event
from typing import Any, Generic, TypeVar

from action_msgs.srv import CancelGoal
from rclpy import Future
from rclpy.action.client import ClientGoalHandle

from node_helpers import futures
from node_helpers.futures import ExceptionCollector
from node_helpers.robust_rpc import RobustActionClient

FeedbackCallback = Callable[[Any], bool]
GOAL = TypeVar("GOAL")
T = TypeVar("T")


@dataclass
class ActionElement(Generic[GOAL]):
    client: RobustActionClient

    goal: GOAL

    feedback_callback: FeedbackCallback | None = None
    """A function that will be hooked up to the feedback callback. The input argument is
    the Feedback message object and the return value should be True if the action is
    ready to trigger the next ActionGroup in the ActionSequence.
    """


class NoRunningActionsError(Exception):
    """Raised when an operation is to cancel or wait for results, but there are no
    running actions"""


class AlreadyRunningActionsError(Exception):
    """Raised when trying to execute on a group that's already running actions"""


def _normalize_iterable(*elements: Iterable[T] | T) -> list[T]:
    """Normalize an iterable of ActionElements into a list"""
    return list(
        itertools.chain.from_iterable(
            list(e) if isinstance(e, Iterable) else [e] for e in elements
        )
    )


class ActionGroup:
    """A helper utility for running multiple actions simultaneously, and waiting for
    completion (or a specified partial completion).
    """

    @dataclass
    class _RunningAction:
        result_future: Future

        goal_handle: ClientGoalHandle
        """The goal_handle for the ongoing action"""

        next_action_trigger: Event
        """Set either when the action is finished, or when the feedback_callback has
        returned True.
        """

    def __init__(
        self, *elements: ActionElement[GOAL] | Iterable[ActionElement[GOAL]]
    ) -> None:
        """Creates the action group and begins the actions
        :param elements: Either a list of elements, or iterables of elements.
            For example, ActionGroup(element_a, element_b) is acceptable, as is
            ActionGroup(ActionElement(..) for client, goal in clients_and_goals)
        :raises ValueError: If no items were passed in to elements.
        """
        if len(elements) == 0:
            # This might be reconsidered, if there's a use case for not raising an error
            raise ValueError("There must be at least one element!")

        # Unpack elements or iterables of elements into a single flat list
        self._action_elements: list[ActionElement[GOAL]] = _normalize_iterable(
            *elements
        )
        self._running_actions: list[ActionGroup._RunningAction] = []

    # APIs that somewhat mirror Action Client
    def send_goals_async(self) -> None:
        """Asynchronously begin running all actions"""
        if len(self._running_actions):
            raise AlreadyRunningActionsError("This function can only be called once!")

        # Start all the actions async and hook up feedback callbacks
        handle_futures: list[Future] = []
        next_action_trigger_events: list[Event] = []
        for element in self._action_elements:
            on_feedback: Callable[[Any], None] | None
            next_action_trigger = Event()
            if element.feedback_callback is not None:

                def on_feedback(
                    feedback: Any,
                    callback: FeedbackCallback = element.feedback_callback,  # type: ignore
                    trigger: Event = next_action_trigger,
                ) -> None:
                    if callback(feedback.feedback):
                        trigger.set()

            else:
                on_feedback = None

            handle_futures.append(
                element.client.send_goal_async(
                    goal=element.goal, feedback_callback=on_feedback
                )
            )
            next_action_trigger_events.append(next_action_trigger)

        # Sync the futures such that now we know all actions were accepted
        handles = futures.wait_for_futures(handle_futures, ClientGoalHandle)

        # Hook up each action so when they finish the feedback_trigger is also called
        for handle, trigger in zip(handles, next_action_trigger_events, strict=False):
            result_future: Future = handle.get_result_async()
            result_future.add_done_callback(lambda f, trigger=trigger: trigger.set())
            action = self._RunningAction(
                next_action_trigger=trigger,
                result_future=result_future,
                goal_handle=handle,
            )
            self._running_actions.append(action)

    def send_goals(self) -> list[Any]:
        """Synchronously run all actions"""
        self.send_goals_async()
        return self.wait_for_results()

    def cancel_goals(self) -> None:
        """Synchronously cancels all (if any) running actions, then blocks until done"""
        if not len(self._running_actions):
            return

        self.cancel_goals_async()
        self.wait_for_results()

    def cancel_goals_async(self) -> None:
        """Synchronously cancels all (if any) running actions, then don't wait"""
        if not len(self._running_actions):
            return

        # Asynchronously trigger cancellation on all running actions
        cancellation_acceptance_futures = [
            handler.goal_handle.cancel_goal_async() for handler in self._running_actions
        ]

        # Wait until all cancellation requests have been (presumably) accepted
        # TODO: If there's ever a need for it, it may be worth checking that the actions
        #       that were cancelled actually accepted the cancellation response.
        futures.wait_for_futures(
            cancellation_acceptance_futures, type_=CancelGoal.Response, timeout=30
        )

    # Additional APIs useful for groups, that don't mirror ActionClient APIs
    def yield_for_results(
        self, yield_interval: float = 0.1
    ) -> Generator[None, None, list[Any]]:
        """This is a helpful method that is equivalent to wait_for_results except it
        yields periodically.

        Usage example in an ActionWorker.run loop:
        >>> group.send_goals_async()
        >>> # The following line allows cancellation while waiting for actions to finish
        >>> action_results = yield from group.yield_for_results()

        :param yield_interval: How often to yield, while waiting for results
        :yields: None, until all futures are complete
        :return: The results of all the actions in the action group
        :raises NoRunningActionsError: If no actions are running
        """

        if len(self._running_actions) == 0:
            raise NoRunningActionsError(
                "You must start the actions before waiting on results!"
            )

        results = yield from futures.yield_for_futures(
            [a.result_future for a in self._running_actions], object, yield_interval
        )

        self._running_actions = []
        return [r.result for r in results]  # type: ignore

    def wait_for_results(self) -> list[Any]:
        """Wait until all actions have returned a result"""

        if len(self._running_actions) == 0:
            raise NoRunningActionsError(
                "You must start the actions before waiting on results!"
            )

        results = futures.wait_for_futures(
            [r.result_future for r in self._running_actions], object
        )
        self._running_actions = []

        # Return the results *.result method, instead of its {Action}_GetResult_Response
        return [r.result for r in results]  # type: ignore

    def wait_for_feedback_triggers(self) -> None:
        """Wait until all actions 'feedback_trigger' events are set"""
        if len(self._running_actions) == 0:
            raise NoRunningActionsError(
                "You must start the actions before waiting on triggers!"
            )

        for running_action in self._running_actions:
            running_action.next_action_trigger.wait()

        # Raise exception if any of the actions have already finished and failed
        self._check_for_exceptions()

    def yield_for_feedback_triggers(self) -> Generator[None, None, None]:
        """Yield until all actions 'feedback_trigger' events are set"""
        if len(self._running_actions) == 0:
            raise NoRunningActionsError(
                "You must start the actions before waiting on triggers!"
            )

        while not all(
            running_action.next_action_trigger.is_set()
            for running_action in self._running_actions
        ):
            yield

        # Raise exception if any of the actions have already finished and failed
        self._check_for_exceptions()

    def _check_for_exceptions(self) -> None:
        """Check for exceptions in the running actions"""
        collector = ExceptionCollector()
        for running_action in self._running_actions:
            if running_action.result_future.done():
                with collector:
                    running_action.result_future.result()

        if collector.exceptions:
            self.wait_for_results()
        collector.maybe_raise()


class ActionSequence:
    """Execute a list of ActionGroups.

    Usage Example:
    >>> sequence = ActionSequence(
    >>>     ActionGroup(
    >>>         ActionElement(client, goal, feedback_callback=ready_to_continue),
    >>>         ActionElement(client, goal, feedback_callback=ready_to_continue),
    >>>     ),
    >>>    ActionGroup(
    >>>         ActionElement(client, goal),
    >>>     )
    >>>)

    In the above code, the first action group will run in parallel until all the
    'ready_to_continue' functions set their respective feedback_trigger events, at
    which point the next ActionGroup will begin to run in parallel.

    If no 'ready_to_continue' function is passed, then the ActionGroups will be run
    one after the other, synchronously (waiting for results from all child actions
    within the group before continuing on to the next group).
    """

    def __init__(self, *action_groups: ActionGroup | Iterable[ActionGroup]) -> None:
        self._action_groups = _normalize_iterable(*action_groups)

    def execute(self) -> list[list[Any]]:
        """Execute all action groups sequentially"""
        for action_group in self._action_groups:
            action_group.send_goals_async()
            try:
                action_group.wait_for_feedback_triggers()
            except Exception:
                action_group.wait_for_results()
                raise

        return self.wait_for_results()

    def yield_for_execution(self) -> Generator[None, None, list[list[Any]]]:
        """Execute all action groups sequentially, yielding periodically"""
        try:
            for action_group in self._action_groups:
                action_group.send_goals_async()
                yield from action_group.yield_for_feedback_triggers()
        except Exception:
            # Block until all actions are finished
            action_group.wait_for_results()
            raise

        # Wait for all actions to finish executing, and raising potential exceptions
        results = []
        for action_group in self._action_groups:
            result = yield from action_group.yield_for_results()
            results.append(result)

        return results

    def cancel_async(self) -> None:
        """Cancel all running actions"""
        for action_group in self._action_groups:
            action_group.cancel_goals_async()

    def cancel(self) -> None:
        """This finishes any ongoing cancellations, and waits for results"""
        # Begin cancelling async
        for action_group in self._action_groups:
            action_group.cancel_goals_async()

        # Block until all actions are cancelled
        for action_group in self._action_groups:
            action_group.cancel_goals()

    def wait_for_results(self) -> list[list[Any]]:
        """Wait until all actions have returned a result or finished cancelling"""
        return [action_group.wait_for_results() for action_group in self._action_groups]


class ParallelActionSequences:
    """Useful when you need to run parallel tracks of ActionSequences.

    For example, let's say you need to run the sequence:
        1. grab nails
        2. go to home position
        3. retract

    But you need to do it for 2 different robots, in parallel. You don't want to
    wait for the first robot to finish grabbing nails before the second robot can
    start grabbing nails. This is where ParallelActionSequences comes in.
    """

    def __init__(
        self, *action_sequences: ActionSequence | Iterable[ActionSequence]
    ) -> None:
        self._action_sequences = _normalize_iterable(*action_sequences)

    def yield_for_execution(self) -> Generator[None, None, list[list[list[Any]]]]:
        """Execute all action sequences in parallel, yielding

        :yield: None, until all action sequences are finished
        :return: A list of where
            - Each element in the list is the result of an ActionSequence
            - Each element in the inner list is the results of actions of an ActionGroup
            - Each element in the inner-inner list is a ROS action Result message
        :raises Exception: If any of the actions raise exceptions, the first will be
            re-raised here.
        """

        # Get generators for each action sequence
        sequence_generators = [
            seq.yield_for_execution() for seq in self._action_sequences
        ]

        # Keep track of whether each generator is done
        done = [False] * len(sequence_generators)
        results = [None] * len(sequence_generators)
        collector = ExceptionCollector()
        while not all(done):
            for seq_idx, gen in enumerate(sequence_generators):
                if not done[seq_idx]:
                    with collector:
                        try:
                            next(gen)
                        except StopIteration as e:
                            results[seq_idx] = e.value
                            done[seq_idx] = True
                        except Exception:
                            done[seq_idx] = True
                            raise

            # Yield inbetween checks of the underlying generators
            yield

        collector.maybe_raise()

        return results  # type: ignore

    def cancel_async(self) -> None:
        """Cancel all running actions"""
        for action_sequence in self._action_sequences:
            action_sequence.cancel_async()

    def cancel(self) -> None:
        """This finishes any ongoing cancellations, and waits for results"""
        self.cancel_async()

        for action_sequence in self._action_sequences:
            action_sequence.cancel()
