"""This module contains common QOS constants used across the stack.

Make sure to document the intended use case for each QOS, so that they can easily be
reused.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSDurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

qos_profile_transient_reliable_keep_all = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL,
)
"""This is used for subscribing/publishing to a topic where receiving all messages ever
published is important.

Keep in mind this should be used for transferring occasional messages, where the number
of messages will not go up to infinity as time goes on. If it does, you will eventually
run out of either storage or RAM.
"""

qos_latching = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
"""An approximation of latching from ROS1. Makes the latest published message available
always.
"""

qos_reliable_latest_msg = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
