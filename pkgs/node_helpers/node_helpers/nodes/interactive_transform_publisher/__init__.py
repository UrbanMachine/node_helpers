import logging
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarkerFeedback

from node_helpers.markers import InteractiveVolumeMarker
from node_helpers.nodes.helpful_node import HelpfulNode
from node_helpers.parameters import param_path
from node_helpers.qos import qos_profile_transient_reliable_keep_all
from node_helpers.ros2_numpy import msgify, numpify
from node_helpers.spinning import create_spin_function
from node_helpers.tf import ConstantStaticTransformBroadcaster

from .exceptions import DuplicateTransformError, MultipleParentsError
from .schemas import TransformDescription, TransformModel, TransformsFile


class InteractiveTransformPublisher(HelpfulNode):
    """
    An interactive transform publisher node for ROS 2 that provides a way to
    interactively manipulate and publish static transforms using RViz.

    This node allows users to easily calibrate and glue URDFs together by exposing
    interactive markers in RViz for manipulating transforms. It reads and writes
    transforms to a specified file, allowing easy persistence of transforms across
    sessions.

    The node exposes the following topics:
    * `/tf_static_updates`: A topic to listen for external updates to transforms.
        If a TF is published here that doesn't yet exist, it will be ignored.
    * `/tf_static_create`: A topic for creating new TFs, if they didn't already exist.
        If a TF is published here that already existed (e.g. from a file or ROS param
        config), no change will be made.

    To use this node, make sure to have an appropriate configuration that specifies
    the required parameters such as static_transforms_file, transforms,
    tf_publish_frequency, and scale_factor.
    """

    def __init__(self, **kwargs: Any):
        super().__init__("interactive_transform_publisher", **kwargs)
        # Set up parameters
        self.transforms_path = param_path(
            self.declare_and_get_parameter(
                "static_transforms_file", type_=str, required=True, description=""
            )
        )
        initial_transforms: list[str] = self.declare_and_get_parameter(
            "transforms",
            type_=list[str],
            required=True,
            description="A list of strings in the format of "
            "'parent_tf_name:child_tf_name', which will turn into zeroed out "
            "transforms that can then be interacted with.",
        )
        self.publish_seconds = 1 / self.declare_and_get_parameter(
            "tf_publish_frequency",
            type_=float,
            required=True,
            description="The frequency (HZ) to publish to TF static",
        )
        self.scale_factor = self.declare_and_get_parameter(
            "scale_factor",
            type_=float,
            default_value=1.0,
            description="How much to scale movements by",
        )

        # Set up the interactive marker server
        self.interaction_server = InteractiveMarkerServer(
            node=self,
            namespace=self.get_namespace(),
        )

        # Set up a receiver for TF changes
        self.create_subscription(
            TransformStamped,
            "tf_static_updates",
            callback=self._on_update_transform,
            qos_profile=qos_profile_transient_reliable_keep_all,
        )

        # Set up a receiver for TF creation (for defining TFs that don't exist in ROS
        # params or in the state file)
        self.create_subscription(
            TransformStamped,
            "tf_static_create",
            callback=self._on_create_transform,
            qos_profile=qos_profile_transient_reliable_keep_all,
        )

        # Register transforms from the state file into self.transforms and the server
        self.transforms: list[TransformDescription] = []
        self._load_state_file(initial_transforms, self.transforms_path)

    def _load_state_file(
        self, allowed_transform_pairs: list[str], transform_path: Path
    ) -> None:
        """Load transforms from a file into self.transforms and the interactive marker
        server.
        This function ensures that 'stale' transforms in the file that are not specified
        in configuration (and were not created via API) are removed from the file.

        :param allowed_transform_pairs: A list of strings of format 'parent_tf:child_tf'
        :param transform_path: A path to a file that follows the pydantic TransformsFile
            schema.
        """

        # First, we start by making identity transforms, as specified by ROS params
        allowed_transforms: dict[str, TransformModel] = {
            key: TransformModel(parent=p, child=c, created_via_api=False)
            for key, p, c in ((s, *s.split(":")) for s in allowed_transform_pairs)
        }
        if transform_path.is_file():
            file_transforms = TransformsFile.model_validate_json(
                transform_path.read_text()
            ).transforms

            # The following logic loads transforms only if either of these are true:
            # 1) They were specified in ROS parameters
            # 2) They were in the transforms file AND marked as having been created
            #    via API
            for loaded_transform in file_transforms:
                key = f"{loaded_transform.parent}:{loaded_transform.child}"
                if key in allowed_transforms:
                    # Ensure that parameter specified transforms are marked as such
                    loaded_transform.created_via_api = False
                    allowed_transforms[key] = loaded_transform
                elif loaded_transform.created_via_api:
                    allowed_transforms[key] = loaded_transform

        for model in allowed_transforms.values():
            self._register_transform(model)

    def _register_transform(self, model: TransformModel) -> TransformDescription:
        # First, validate this transform isn't a duplicate that's already been added
        existing_marker_names: set[str] = {t.model.marker_name for t in self.transforms}
        if model.marker_name in existing_marker_names:
            raise DuplicateTransformError(
                f"Tried to register {model.marker_name}, which already exists!"
            )

        # Next, validate this transforms child doesn't already have a registered parent
        existing_children: set[str] = {t.model.child for t in self.transforms}
        if model.child in existing_children:
            raise MultipleParentsError(
                f"Tried to register {model.child} as a child to {model.parent}, but "
                f"it already has a different parent configured in the interactive "
                f"transforms configuration! It's possible this resulted from changing "
                f"yaml configuration multiple times with different transforms. "
                f"To fix this You can either edit the configuration file directly, or "
                f"delete it and relaunch the stack.\n"
                f"Bad configuration file path: {self.transforms_path}"
            )

        # Create a ConstantStaticTransformPublisher, and reference it in self.transforms
        transform_description = TransformDescription(
            model=model,
            broadcaster=ConstantStaticTransformBroadcaster(
                self, publish_seconds=self.publish_seconds
            ),
        )
        self.transforms.append(transform_description)

        # Set the transform for the broadcaster
        self.get_logger().info(f"Publishing static transform {model}")
        transform_description.broadcaster.set_transform(model.to_msg(stamp=Time()))

        # Add this to the interactive transform server
        self._insert_interactive_transform(transform_description)
        return transform_description

    def _insert_interactive_transform(
        self, transform_description: TransformDescription
    ) -> None:
        """Insert a transform without applying changes on the server"""
        transform = transform_description.model
        marker = InteractiveVolumeMarker(
            name=transform_description.model.marker_name,
            description=transform.child,
            scale=[0.05, 0.05, 0.05],
            frame_id=transform.parent,
            position=msgify(Point, np.array(transform.translation) / self.scale_factor),
            orientation=msgify(Quaternion, np.array(transform.rotation)),
        )
        self.interaction_server.insert(
            marker.interactive_marker,
            feedback_callback=partial(self._feedback, transform_description),
        )
        self.interaction_server.applyChanges()

    def _feedback(
        self,
        transform_description: TransformDescription,
        feedback: InteractiveMarkerFeedback,
    ) -> None:
        """Called when a user moves a transform in rviz"""
        transform = transform_description.model

        # Get the interpolated translation
        position = numpify(feedback.pose.position)
        previous_position = transform.translation
        pos_delta = position - previous_position
        interpolated_transform = (
            (previous_position + pos_delta) * self.scale_factor
        ).tolist()

        # Publish a static transform
        transform.rotation = tuple(numpify(feedback.pose.orientation).tolist())
        transform.translation = tuple(interpolated_transform)
        msg = transform.to_msg(stamp=self.get_clock().now().to_msg())

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._save()

        transform_description.broadcaster.set_transform(msg)

    def _on_update_transform(self, transform_msg: TransformStamped) -> None:
        """Publishes a transform and saves it to the file"""

        # Try to find a matching transform description to update
        try:
            transform_description = next(
                t
                for t in self.transforms
                if t.model.parent == transform_msg.header.frame_id
                and t.model.child == transform_msg.child_frame_id
            )
        except StopIteration:
            self.get_logger().error(
                "The transform description does not exist for the parent frame "
                f"'{transform_msg.header.frame_id}' and child frame "
                f"'{transform_msg.child_frame_id}'. If you would like to create a new "
                f"transform via API, publish it on the 'tf_static_create' topic."
            )
            return

        # Edit parameters in place
        rotation_np = numpify(transform_msg.transform.rotation)
        translation_np = numpify(transform_msg.transform.translation)
        transform_description.model.rotation = tuple(rotation_np.tolist())
        transform_description.model.translation = tuple(translation_np.tolist())

        # Save changes to file
        self._save()

        # Update the interactive marker so it reflects the new state.
        # This prevents a bug where after automatic calibration, when a user touches
        # a transform on RVIZ it would 'reset' back to the position last known by the
        # interaction server.
        marker_name = transform_description.model.marker_name
        assert self.interaction_server.setPose(
            transform_description.model.marker_name,
            pose=Pose(
                orientation=msgify(Quaternion, rotation_np),
                position=msgify(Point, translation_np / self.scale_factor),
            ),
        )
        self.interaction_server.applyChanges()

        # Send the transform
        transform_description.broadcaster.set_transform(transform_msg)
        self.get_logger().info(f"Successfully updated the transform '{marker_name}'")

    def _on_create_transform(self, transform_msg: TransformStamped) -> None:
        """Create a transform if it didn't already exist, otherwise modify nothing."""
        try:
            self._register_transform(
                TransformModel(
                    created_via_api=True,
                    parent=transform_msg.header.frame_id,
                    child=transform_msg.child_frame_id,
                    rotation=tuple(numpify(transform_msg.transform.rotation).tolist()),
                    translation=tuple(
                        numpify(transform_msg.transform.translation).tolist()
                    ),
                )
            )
        except DuplicateTransformError:
            self.get_logger().info(
                f"Not creating transform {transform_msg}, it already exists"
            )
        else:
            self._on_update_transform(transform_msg)

    def _save(self) -> None:
        """Update the transforms file"""
        self.get_logger().error(f"Saving transforms to file: {self.transforms_path}")
        self.transforms_path.write_text(
            TransformsFile(
                transforms=[t.model for t in self.transforms]
            ).model_dump_json()
        )


main = create_spin_function(InteractiveTransformPublisher, multi_threaded=True)
