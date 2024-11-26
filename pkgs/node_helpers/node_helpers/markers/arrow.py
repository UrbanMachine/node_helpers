from typing import Any

import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker


def create_point_to_point_arrow_marker(
    shaft_diameter: float,
    head_diameter: float,
    head_length: float,
    base_point: tuple[float, float, float] | npt.NDArray[np.float64] | Vector3 | Point,
    head_point: tuple[float, float, float] | npt.NDArray[np.float64] | Vector3 | Point,
    **marker_kwargs: Any,
) -> Marker:
    """Create an arrow marker using a base point to a head point.

    :param shaft_diameter: The diameter of the arrow shaft
    :param head_diameter: The diameter of the arrow head
    :param head_length: The length of the arrow head
    :param base_point: The base point of the arrow
    :param head_point: The head point of the arrow
    :param marker_kwargs: The arguments to pass on to the Marker

    :return: The arrow marker
    """
    if isinstance(base_point, Vector3 | Point):
        base_point = (base_point.x, base_point.y, base_point.z)
    if isinstance(head_point, Vector3 | Point):
        head_point = (head_point.x, head_point.y, head_point.z)

    return Marker(
        type=Marker.ARROW,
        # x: Shaft diameter, Y: head diameter, Z: head length, as per:
        # http://wiki.ros.org/rviz/DisplayTypes/Marker#Arrow_.28ARROW.3D0.29
        scale=Vector3(x=shaft_diameter, y=head_diameter, z=head_length),
        points=[Point(x=p[0], y=p[1], z=p[2]) for p in [base_point, head_point]],
        **marker_kwargs,
    )
