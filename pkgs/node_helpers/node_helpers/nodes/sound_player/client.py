from node_helpers_msgs.msg import PlaySound
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_services_default

from node_helpers.nodes import HelpfulNode


class SoundPlayerClient:
    """A helper object for calling APIs on the SoundPlayer"""

    def __init__(
        self,
        node: HelpfulNode,
        namespace: str | None = None,
        callback_group: CallbackGroup | None = None,
    ):
        callback_group = callback_group or MutuallyExclusiveCallbackGroup()
        self.namespace = namespace or node.get_namespace().replace("/", "")

        self.play_sound = node.create_publisher(
            PlaySound,
            f"/{self.namespace}/play_sound",
            qos_profile=qos_profile_services_default,
            callback_group=callback_group,
        )
