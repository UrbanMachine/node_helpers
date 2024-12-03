import queue
from threading import Event

from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from node_helpers.nodes import HelpfulNode
from node_helpers.robust_rpc.typing import RobustActionMsg

from ._typing import FEEDBACK_TYPE, GOAL_TYPE, RESULT_TYPE
from .base_handler import ActionHandler
from .worker import ActionWorker


class QueuedActionHandler(ActionHandler[GOAL_TYPE, FEEDBACK_TYPE, RESULT_TYPE]):
    """Gives requests to ActionWorkers one-by-one, ensuring the action is not run more
    than once at a time
    """

    def __init__(
        self,
        node: HelpfulNode,
        action_name: str,
        action_type: type[RobustActionMsg],
        cancellable: bool = True,
    ):
        super().__init__(
            node=node,
            action_name=action_name,
            action_type=action_type,
            callback_group=ReentrantCallbackGroup(),
            cancellable=cancellable,
        )

        self._current_worker: (
            None | (ActionWorker[GOAL_TYPE, FEEDBACK_TYPE, RESULT_TYPE])
        ) = None

        # A queue of workers waiting to run, and an event used to signal when it is
        # time for that worker to run
        self._worker_queue: "queue.Queue[tuple[ActionWorker[GOAL_TYPE, FEEDBACK_TYPE, RESULT_TYPE], Event]]" = (  # noqa: E501
            queue.Queue()
        )

        self._find_work_timer = node.create_timer(
            0.05, self._find_work, callback_group=self._callback_group
        )

    def is_working(self) -> bool:
        """
        :return: If the handler is currently processing a request
        """
        return self._current_worker is not None

    def on_goal(self, goal_handle: ServerGoalHandle) -> RESULT_TYPE:
        worker = self.create_worker(goal_handle)

        queued_actions = self._worker_queue.qsize()
        if queued_actions > 0:
            self.node.get_logger().warning(
                f"{queued_actions} actions waiting on the job queue for "
                f"{self.action_name}. Requests may be delayed"
            )

        start_working = Event()

        self._worker_queue.put((worker, start_working))

        # Wait until it's this worker's turn to run
        start_working.wait()

        return worker.execute_callback(self._request_timeout)

    def _find_work(self) -> None:
        """Looks for new requests in the action queue, if no action is currently being
        run
        """

        if self._current_worker is not None and self._current_worker.done:
            self._current_worker = None

        if self._current_worker is None:
            try:
                worker, start_executing = self._worker_queue.get_nowait()
                self._current_worker = worker
                start_executing.set()
            except queue.Empty:
                pass
