from collections.abc import Generator
from pathlib import Path
from tempfile import TemporaryDirectory

from rclpy import Parameter

from node_helpers.nodes import InteractiveTransformPublisher
from node_helpers.testing import set_up_node


def set_up_static_transforms(
    *parents_to_children: tuple[str, str], namespace: str = "calibration"
) -> Generator[InteractiveTransformPublisher, None, None]:
    """This function can be used to create and publish static transforms in a fixture.
    It operates much like set_up_node. Here's how to use it:

    >>> @pytest.fixture
    >>> def world_wood_robot_transforms():
    >>>     yield from set_up_static_transforms(
    >>>         ("world", "base_link"),
    >>>         ("base_link", "wood"),
    >>>         ("base_link", "robot"),
    >>>     )

    The above example would create static transforms between each parent and child.

    :param parents_to_children: The parent -> child TF pairs to create
    :param namespace: The namespace to use for the node
    :yields: A configured, launched InteractiveTransformPublisher node
    """
    parents_to_children = (
        parents_to_children if parents_to_children else (("world", "base_link"),)
    )
    transforms_str = [f"{parent}:{child}" for parent, child in parents_to_children]

    with TemporaryDirectory() as tempdir:
        temp_config = Path(tempdir) / "transforms.json"
        yield from set_up_node(
            node_class=InteractiveTransformPublisher,
            namespace=namespace,
            multi_threaded=True,
            node_name="static_transforms",
            parameter_overrides=[
                Parameter(name="static_transforms_file", value=str(temp_config)),
                Parameter(name="tf_publish_frequency", value=2.0),
                Parameter(name="transforms", value=transforms_str),
            ],
        )
