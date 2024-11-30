import contextlib
import logging
from collections.abc import Generator
from queue import Empty
from tempfile import NamedTemporaryFile
from typing import Any

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from interactive_markers.interactive_marker_server import MarkerContext
from node_helpers.nodes.interactive_transform_publisher import (
    DuplicateTransformError,
    InteractiveTransformPublisher,
    MultipleParentsError,
    TransformModel,
    TransformsFile,
)
from node_helpers.nodes.interactive_transform_publisher.client import (
    InteractiveTransformClient,
)
from node_helpers.ros2_numpy import msgify, numpify
from node_helpers.testing import NodeForTesting, set_up_node
from rclpy import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    qos_profile_services_default,
)
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerFeedback

_DEFAULT_TRANSFORMS = ["april_tag:april_tag_origin", "cool_parent:cool_child"]


class TransformClient(NodeForTesting, InteractiveTransformClient):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__("transform_client", **kwargs)  # type: ignore
        InteractiveTransformClient.__init__(self, self)

        self.tf_static = self.create_queue_subscription(
            type_=TFMessage,
            topic="/tf_static",
            qos_profile=QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_ALL,
            ),
        )
        self.feedback = self.create_publisher(
            InteractiveMarkerFeedback,
            "feedback",
            qos_profile=qos_profile_services_default,
        )


_TRANSFORM_1 = TransformModel(
    parent="april_tag",
    child="april_tag_origin",
    created_via_api=False,
    translation=(1.0, 2.0, 3.0),
    rotation=(0.1, 2.0, 0.3, 1.0),
)
_TRANSFORM_2 = TransformModel(
    parent="cool_parent", child="cool_child", created_via_api=False
)
_TRANSFORM_3 = TransformModel(
    parent="darn", child="diggity", created_via_api=True, translation=(7.0, 8.0, 9.0)
)
_TRANSFORMS_FILE = TransformsFile(transforms=[_TRANSFORM_1, _TRANSFORM_2, _TRANSFORM_3])


@contextlib.contextmanager
def set_up_interactive_transform_publisher(
    rosparam_transforms: list[str],
) -> Generator[InteractiveTransformPublisher, None, None]:
    """Start an InteractiveTransformPublisher with a tempdir and specific config"""
    with NamedTemporaryFile("w") as config_file:
        config_file.write(_TRANSFORMS_FILE.model_dump_json())
        config_file.flush()

        generator = set_up_node(
            InteractiveTransformPublisher,
            "calibration",
            "interactive_transform_publisher",
            parameter_overrides=[
                Parameter(name="static_transforms_file", value=config_file.name),
                Parameter(name="transforms", value=rosparam_transforms),
                # Disable constant re-publishing of TFs to make testing easier
                Parameter(name="tf_publish_frequency", value=0.00001),
            ],
        )

        try:
            yield next(generator)
        finally:
            tuple(generator)


@pytest.fixture()
def transform_publisher() -> Generator[InteractiveTransformPublisher, None, None]:
    with set_up_interactive_transform_publisher(_DEFAULT_TRANSFORMS) as publisher:
        yield publisher


@pytest.fixture()
def transform_client() -> Generator[TransformClient, None, None]:
    yield from set_up_node(TransformClient, "calibration", "transform_client")


def test_initialization(
    transform_publisher: InteractiveTransformPublisher,
    transform_client: TransformClient,
) -> None:
    """Test the contents of the __init__, which should load from a file, create a
    marker server, and publish static transforms."""
    transform_models = [t.model for t in transform_publisher.transforms]
    assert transform_models == [_TRANSFORM_1, _TRANSFORM_2, _TRANSFORM_3]
    assert len(transform_publisher.interaction_server.marker_contexts) == 3

    for transform_description in transform_publisher.transforms:
        transform = transform_description.model

        marker_name = transform_description.model.marker_name
        marker_context: MarkerContext = (
            transform_publisher.interaction_server.marker_contexts[marker_name]
        )
        marker = marker_context.int_marker
        assert marker.header.frame_id == transform.parent
        assert np.allclose(
            numpify(marker.pose.position),
            np.array(transform.translation) / transform_publisher.scale_factor,
        )
        assert np.allclose(numpify(marker.pose.orientation), transform.rotation)

    # Validate static transform are published, and they match expectations
    for transform_description in transform_publisher.transforms:
        tf_message = transform_client.tf_static.get(timeout=10)
        transform = transform_description.model

        assert len(tf_message.transforms) == 1
        published_tf: TransformStamped = tf_message.transforms[0]
        assert published_tf.header.frame_id == transform.parent
        assert published_tf.child_frame_id == transform.child
        assert np.allclose(
            numpify(published_tf.transform.translation),
            transform.translation,
        )
        assert np.allclose(numpify(published_tf.transform.rotation), transform.rotation)


def test_duplicate_transforms(
    transform_publisher: InteractiveTransformPublisher,
) -> None:
    some_new_transform = TransformModel(
        parent="oh im a cool parent, for sure",
        child="stop it! you're embarrassing me!",
        created_via_api=False,
    )
    # No exception
    transform_publisher._register_transform(some_new_transform)

    transforms_before = transform_publisher.transforms.copy()

    # The duplicate should fail
    with pytest.raises(DuplicateTransformError):
        transform_publisher._register_transform(some_new_transform)

    # Nothing should have changed
    assert transforms_before == transform_publisher.transforms


def test_multiple_parents_not_allowed(
    transform_publisher: InteractiveTransformPublisher,
) -> None:
    transform_a = TransformModel(
        parent="now now child, it's important you don't go off with strangers",
        child="impressionable child",
        created_via_api=False,
    )
    transform_b_with_different_parent = TransformModel(
        parent="stranger (oh no, danger!)",
        child="impressionable child",
        created_via_api=False,
    )

    # No exception
    transform_publisher._register_transform(transform_a)

    transforms_before = transform_publisher.transforms.copy()

    # The duplicate should fail
    with pytest.raises(MultipleParentsError):
        transform_publisher._register_transform(transform_b_with_different_parent)

    # Nothing should have changed
    assert transforms_before == transform_publisher.transforms


def test_feedback(
    transform_publisher: InteractiveTransformPublisher,
    transform_client: TransformClient,
) -> None:
    """Test functionality in the feedback callback."""
    expected_rotation_1 = np.array((0.4, 0.2, 0.3, 0.5))
    expected_translation_1 = np.array((1.0, 1.1, 1.2))
    expected_marker_name = _TRANSFORM_1.marker_name
    published_feedback = InteractiveMarkerFeedback(
        marker_name=expected_marker_name,
        pose=Pose(
            position=msgify(Point, expected_translation_1),
            orientation=msgify(Quaternion, expected_rotation_1),
        ),
        event_type=InteractiveMarkerFeedback.POSE_UPDATE,
    )

    # First clear the transforms that were published in the __init__
    for _ in _TRANSFORMS_FILE.transforms:
        transform_client.tf_static.get(timeout=5)

    # Publish feedback and validate the response
    transform_client.feedback.publish(published_feedback)
    retrieved_transforms = transform_client.tf_static.get(timeout=10)
    assert len(retrieved_transforms.transforms) == 1

    # Validate that the feedback published an updated matching static transform
    # except it's been modified by the scale_factor parameters
    transform: TransformStamped = retrieved_transforms.transforms[0]
    assert transform.header.frame_id == _TRANSFORM_1.parent
    assert transform.header.stamp.sec != 0
    assert np.allclose(
        numpify(transform.transform.translation),
        numpify(published_feedback.pose.position) * transform_publisher.scale_factor,
    )
    assert np.allclose(
        numpify(transform.transform.rotation),
        numpify(published_feedback.pose.orientation),
    )

    # Validate that the published feedback modified the internal self.transforms
    assert (
        transform_publisher.transforms[1].model == _TRANSFORM_2
    ), "No change expected!"
    assert np.allclose(
        transform_publisher.transforms[0].model.rotation, expected_rotation_1
    )
    transform_data = transform_publisher.transforms_path.read_text()
    assert (
        TransformsFile.model_validate_json(transform_data) == _TRANSFORMS_FILE
    ), "The file should only be updated on a MOUSE_UP event!"

    # Now publish the exact same feedback, but with a MOUSE_UP event, and validate that
    # the file was updated to reflect the new change.
    published_feedback.event_type = InteractiveMarkerFeedback.MOUSE_UP
    transform_client.feedback.publish(published_feedback)
    # Wait for the static transform to get published
    transform_client.tf_static.get(timeout=5)
    transforms_data = transform_publisher.transforms_path.read_text()
    loaded_file = TransformsFile.model_validate_json(transforms_data)
    assert loaded_file == TransformsFile(
        transforms=[
            TransformModel(
                parent=_TRANSFORM_1.parent,
                child=_TRANSFORM_1.child,
                created_via_api=False,
                rotation=tuple(expected_rotation_1.tolist()),
                translation=tuple(
                    (expected_translation_1 * transform_publisher.scale_factor).tolist()
                ),
            ),
            _TRANSFORM_2,
            _TRANSFORM_3,
        ]
    )


def test_publishing_invalid_transform(
    transform_publisher: InteractiveTransformPublisher,
    transform_client: TransformClient,
) -> None:
    """Test that when 'updating' an untracked parent->child, nothing changed"""
    # First clear the transforms that were published in the __init__
    for _ in _TRANSFORMS_FILE.transforms:
        transform_client.tf_static.get(timeout=5)

    # Now try publishing an invalid transform
    file_before = transform_publisher.transforms_path.read_text()
    model = TransformModel(
        parent="untracked_p", child="untracked_c", created_via_api=False
    )
    transform_client.update_transform.publish(model.to_msg(Time()))
    with pytest.raises(Empty):
        transform_client.tf_static.get(timeout=1)
    assert transform_publisher.transforms_path.read_text() == file_before


def test_on_update_transform(
    transform_publisher: InteractiveTransformPublisher,
    transform_client: TransformClient,
) -> None:
    """Test that when a valid transform is published, that it is then saved to the file
    and republished under tf_static"""
    # First clear the transforms that were published in the __init__
    for _ in _TRANSFORMS_FILE.transforms:
        transform_client.tf_static.get(timeout=5)

    with pytest.raises(Empty):
        transform_client.tf_static.get_nowait()

    published = TransformModel(
        parent=_TRANSFORM_1.parent,
        child=_TRANSFORM_1.child,
        created_via_api=False,
        rotation=(0, 1, 2, 3),
        translation=(2, 5, 6),
    )

    # Publish the tf static update
    transform_client.update_transform.publish(published.to_msg(Time(sec=3234)))

    # Validate the same message was re-published on tf_static
    tf_static_publish = transform_client.tf_static.get(timeout=1)
    assert tf_static_publish == TFMessage(transforms=[published.to_msg(Time(sec=3234))])

    # Validate the file was also updated
    transforms_data = transform_publisher.transforms_path.read_text()
    loaded_file = TransformsFile.model_validate_json(transforms_data)
    assert loaded_file == TransformsFile(
        transforms=[published, _TRANSFORM_2, _TRANSFORM_3]
    )


def test_on_create_transform(
    transform_publisher: InteractiveTransformPublisher,
    transform_client: TransformClient,
) -> None:
    """Test the 'tf_static_create' topic creates only once, and ignores further inputs
    for the same transform."""

    # First clear the transforms that were published in the __init__
    for _ in _TRANSFORMS_FILE.transforms:
        transform_client.tf_static.get(timeout=5)

    published = TransformModel(
        parent="p", child="c", created_via_api=True, rotation=(0, 1, 2, 3)
    )

    # Publish the tf static creation
    transform_client.create_transform.publish(published.to_msg(Time(sec=5819)))

    # This should publish two messages. One with the time == Time() and another with the
    # correct time. This behavior might be helpful, I'm not sure.
    tf_static_publish = transform_client.tf_static.get(timeout=1)
    assert tf_static_publish == TFMessage(transforms=[published.to_msg(Time())])

    tf_static_publish = transform_client.tf_static.get(timeout=1)
    assert tf_static_publish == TFMessage(transforms=[published.to_msg(Time(sec=5819))])

    # Validate the file was also updated, with created_via_api=True
    transforms_data = transform_publisher.transforms_path.read_text()
    loaded_file = TransformsFile.model_validate_json(transforms_data)
    assert loaded_file == TransformsFile(
        transforms=[_TRANSFORM_1, _TRANSFORM_2, _TRANSFORM_3, published]
    )

    # Try publishing again with a different translation, and validate nothing changed
    modified_transform = published.model_copy()
    modified_transform.translation = (1.0, 5.0, 3.0)
    transform_client.create_transform.publish(published.to_msg(Time(sec=10001)))

    # Nothing should have been published
    with pytest.raises(Empty):
        transform_client.tf_static.get(timeout=0.1)

    # The file shouldn't have changed
    assert loaded_file == TransformsFile(
        transforms=[_TRANSFORM_1, _TRANSFORM_2, _TRANSFORM_3, published]
    )


@pytest.mark.parametrize(
    ("transforms_param", "expected_transforms"),
    (
        # Test specifying tf that doesn't exist in file
        (
            [*_DEFAULT_TRANSFORMS, "new_tf_parent:new_tf_child"],
            [
                _TRANSFORM_1,
                _TRANSFORM_2,
                _TRANSFORM_3,
                TransformModel(
                    parent="new_tf_parent", child="new_tf_child", created_via_api=False
                ),
            ],
        ),
        # Test specifying less tf's than exist in file (TF3 should exist because it was
        # created by API, not by ros params)
        ([_DEFAULT_TRANSFORMS[1]], [_TRANSFORM_2, _TRANSFORM_3]),
        # Test exact tfs as those that exist in file
        (_DEFAULT_TRANSFORMS, _TRANSFORMS_FILE.transforms),
        # Test completely different transforms than what's in the file
        (
            ["a_parent:a_child", "amongus:amogus"],
            [
                TransformModel(
                    parent="a_parent", child="a_child", created_via_api=False
                ),
                TransformModel(parent="amongus", child="amogus", created_via_api=False),
                _TRANSFORM_3,
            ],
        ),
    ),
)
def test_state_recovery(
    transforms_param: list[str], expected_transforms: list[TransformModel]
) -> None:
    """Test that the InteractiveTransformPublisher loads transforms as expected"""
    with set_up_interactive_transform_publisher(transforms_param) as publisher:
        publisher_transform_models = [t.model for t in publisher.transforms]
        publisher_transform_models.sort(key=lambda t: t.marker_name)
        expected_transforms.sort(key=lambda t: t.marker_name)
        assert publisher_transform_models == expected_transforms


def test_on_publish_transform_updates_marker_server_state(
    transform_publisher: InteractiveTransformPublisher,
    transform_client: TransformClient,
) -> None:
    """This test verifies there's no longer a bug in the InteractiveTransformPublisher
    where if a transform is modified using `_on_update_transform`, the marker server
    won't know about it, so Rviz will show stale transform positions.
    """
    published = TransformModel(
        parent=_TRANSFORM_1.parent,
        child=_TRANSFORM_1.child,
        created_via_api=False,
        rotation=(0, -0.755, 0.252, 0.606),
        translation=(1, 0, 0),
    ).to_msg(Time())

    # First clear the transforms that were published in the __init__
    for _ in _TRANSFORMS_FILE.transforms:
        transform_client.tf_static.get(timeout=10)

    # Publish the transform and verify it made it to the other side as expected
    transform_client.update_transform.publish(published)
    retrieved_transforms = transform_client.tf_static.get(timeout=10).transforms
    assert len(retrieved_transforms) == 1
    assert np.isclose(
        numpify(retrieved_transforms[0].transform),
        numpify(published.transform),
    ).all()

    # Ensure that the marker server has the updated transform
    marker_name = _TRANSFORM_1.marker_name
    context = transform_publisher.interaction_server.marker_contexts[marker_name]
    marker: InteractiveMarker = context.int_marker
    assert len(transform_publisher.interaction_server.marker_contexts) == 3
    assert np.isclose(
        numpify(marker.pose.orientation), numpify(published.transform.rotation)
    ).all()
    assert np.isclose(
        numpify(marker.pose.position),
        numpify(published.transform.translation) / transform_publisher.scale_factor,
    ).all()
