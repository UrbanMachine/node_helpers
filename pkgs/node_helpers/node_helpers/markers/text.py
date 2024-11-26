from typing import Any

from geometry_msgs.msg import Pose, Vector3
from visualization_msgs.msg import Marker


def create_floating_text(
    text: str, text_height: float, pose: Pose, **marker_kwargs: Any
) -> Marker:
    return Marker(
        type=Marker.TEXT_VIEW_FACING,
        # Spaces are replaced with underscores because of a weird rviz bug where
        # spaces make for absolutely gargantuan spaces between words
        text=text.replace(" ", "_"),
        scale=Vector3(z=text_height),
        pose=pose,
        **marker_kwargs,
    )
