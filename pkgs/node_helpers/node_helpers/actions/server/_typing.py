from typing import TypeVar

from node_helpers.robust_rpc.typing import ResponseType

GOAL_TYPE = TypeVar("GOAL_TYPE")
FEEDBACK_TYPE = TypeVar("FEEDBACK_TYPE")
RESULT_TYPE = TypeVar("RESULT_TYPE", bound=ResponseType)
