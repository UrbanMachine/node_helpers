from visualization_msgs.msg import Marker, MarkerArray


def ascending_id_marker_array(
    markers: list[Marker],
    start_id: int = 0,
    delete_existing: bool = True,
    marker_namespace: str = "",
) -> MarkerArray:
    """Return the same list of markers modified in-place so that each marker has
    ascending IDs.
    :param markers: The markers to modify in-place
    :param start_id: The ID to begin at
    :param delete_existing: If True, all markers for this topic will be cleared before
        the new markers are added
    :param marker_namespace: The namespace to create markers under
    :returns: A MarkerArray with markers that have ascending IDs
    """

    if delete_existing:
        markers.insert(0, Marker(action=Marker.DELETEALL, ns=marker_namespace))

    for i, marker in enumerate(markers, start=start_id):
        marker.id = i
        marker.ns = marker_namespace
    return MarkerArray(markers=markers)
