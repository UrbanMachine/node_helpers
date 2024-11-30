from rclpy.node import Node

from node_helpers.destruction import DestroyCallbacksMixin
from node_helpers.parameters import ParameterMixin
from node_helpers.robust_rpc import RobustRPCMixin
from node_helpers.timing import SingleShotMixin, TimerWithWarningsMixin


class HelpfulNode(
    ParameterMixin,
    RobustRPCMixin,
    DestroyCallbacksMixin,
    SingleShotMixin,
    TimerWithWarningsMixin,
    Node,
):
    """This node class combines all helper mixins within node_helpers into one node"""
