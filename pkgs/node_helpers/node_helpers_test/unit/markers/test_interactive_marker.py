from node_helpers.markers import InteractiveVolumeMarker


def test_init_configuration() -> None:
    """Just basic testing that the object is built correctly"""
    name = "cool_name"
    frame_id = "cool_frame"
    scale = [1.0, 2.0, 3.0]
    description = "cool description"

    marker = InteractiveVolumeMarker(
        name=name, frame_id=frame_id, scale=scale, description=description, fixed=False
    )
    assert marker.interactive_marker.name == name
    assert marker.interactive_marker.description == description

    # The frame_id should be passed to both the interactive_marker and it's box
    assert marker.interactive_marker.header.frame_id == frame_id
    assert marker.box.header.frame_id == frame_id

    # The scale parameter should refer to the underlying box
    box_scale = [marker.box.scale.x, marker.box.scale.y, marker.box.scale.z]
    assert box_scale == scale

    marker_scale = marker.interactive_marker.scale
    assert marker_scale == 3 * 2.25
    assert (
        len(marker.interactive_marker.controls) == 7
    ), "There should be 7 controllers, 1 for the box, and 6 since it's a 6 dof marker!"
