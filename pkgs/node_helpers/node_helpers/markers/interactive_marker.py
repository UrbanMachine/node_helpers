from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class InteractiveVolumeMarker:
    _axis = [
        (
            Quaternion(w=1.0, x=1.0, y=0.0, z=0.0),
            "rotate_x",
            InteractiveMarkerControl.ROTATE_AXIS,
        ),
        (
            Quaternion(w=1.0, x=0.0, y=1.0, z=0.0),
            "rotate_z",
            InteractiveMarkerControl.ROTATE_AXIS,
        ),
        (
            Quaternion(w=1.0, x=0.0, y=0.0, z=1.0),
            "rotate_y",
            InteractiveMarkerControl.ROTATE_AXIS,
        ),
        (
            Quaternion(w=1.0, x=1.0, y=0.0, z=0.0),
            "move_x",
            InteractiveMarkerControl.MOVE_AXIS,
        ),
        (
            Quaternion(w=1.0, x=0.0, y=1.0, z=0.0),
            "move_z",
            InteractiveMarkerControl.MOVE_AXIS,
        ),
        (
            Quaternion(w=1.0, x=0.0, y=0.0, z=1.0),
            "move_y",
            InteractiveMarkerControl.MOVE_AXIS,
        ),
    ]

    def __init__(
        self,
        name: str,
        frame_id: str,
        scale: list[float],
        description: str = "",
        position: Point = None,
        orientation: Quaternion = None,
        fixed: bool = False,
        show_6dof: bool = True,
    ):
        """
        :param name: The visual name of the marker
        :param frame_id: The frame ID the marker will move inside
        :param scale: The [x, y, z] scaling of the marker
        :param description: The description that will show up in rviz
        :param position: The initial position of the marker
        :param orientation: The initial orientation of the marker.
        :param fixed: Whether the marker is movable
        :param show_6dof: Whether to display the 6dof controls
        """
        initial_pose = Pose(
            position=position or Point(), orientation=orientation or Quaternion()
        )

        self.box = Marker(
            scale=Vector3(x=scale[0], y=scale[1], z=scale[2]),
            color=ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),
            header=Header(frame_id=frame_id),
            pose=initial_pose,
            type=Marker.CUBE,
        )

        self.box_control = InteractiveMarkerControl(
            always_visible=True, markers=[self.box]
        )

        self.interactive_marker = InteractiveMarker(
            pose=initial_pose,
            # Make the controls encompass the whole of the box
            scale=max(scale) * 2.25,
            header=self.box.header,
            name=name,
            description=description,
            controls=[self.box_control],
        )

        # Generate 6dof controls
        if show_6dof:
            for (
                orientation,
                control_name,
                axis_interaction,
            ) in self._axis:
                control = InteractiveMarkerControl(
                    orientation=orientation,
                    name=control_name,
                    interaction_mode=axis_interaction,
                )
                if fixed:
                    control = InteractiveMarkerControl.FIXED
                self.interactive_marker.controls.append(control)
