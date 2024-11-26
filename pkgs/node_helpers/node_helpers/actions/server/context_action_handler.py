from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_services_default

from node_helpers.nodes import HelpfulNode
from node_helpers.robust_rpc.typing import RobustActionMsg

from ._typing import FEEDBACK_TYPE, GOAL_TYPE, RESULT_TYPE
from .base_handler import ActionHandler


class ContextActionHandler(ActionHandler[GOAL_TYPE, FEEDBACK_TYPE, RESULT_TYPE]):
    """This is a preconfigured action handler for making context handler actions.
    Specifically, it ensures that the feedback_pub_qos_profile is set to
    services_default, and that the action is cancellable."""

    def __init__(
        self,
        node: HelpfulNode,
        action_name: str,
        action_type: type[RobustActionMsg],
    ):
        super().__init__(
            node=node,
            action_name=action_name,
            action_type=action_type,
            callback_group=ReentrantCallbackGroup(),
            cancellable=True,
            feedback_pub_qos_profile=qos_profile_services_default,
        )
