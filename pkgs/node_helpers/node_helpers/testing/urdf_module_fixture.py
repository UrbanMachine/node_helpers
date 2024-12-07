from collections.abc import Generator
from contextlib import contextmanager
from dataclasses import dataclass
from itertools import chain
from typing import Any, TypeVar

import tf2_py
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from node_helpers.launching.urdf_module_launching import URDFModuleNodeFactory
from node_helpers.nodes import HelpfulNode
from node_helpers.testing.nodes import (
    set_up_external_nodes_from_launchnode,
    set_up_node,
)
from node_helpers.tf import ConstantStaticTransformBroadcaster


class URDFFixtureSetupFailed(Exception):
    """
    Once upon a time, there was an era of about 6 months where tests that relied on
    URDFs would occasionally fail with the error "tf2.LookupException". This exception
    initially would happen somewhere inside the test, and later we started checking if
    the frames were transformable before the test started. This moved the flakiness to
    the setup of the test, where the frames would sometimes not be transformable.

    Why. WHY. WHY are frames sometimes, rarely, not transformable? We don't know. All
    we know is that sometimes in tests the Joint state publisher and Robot state
    publisher somehow don't quite work the way they should, and don't publish frames,
    and thus the TF buffer also doesn't have the frames.

    So, to fix this issue, we added a retry mechanism to URDFModuleFixture.set_up, so
    it will retry up to 3 times to set up the URDFModuleFixture. If it still fails, it
    will raise this exception, which will cause the test to fail.

    Hopefully, this problem will go away in future ROS releases.......
    """


class TFClient(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__("tf_node", **kwargs)
        self.buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.buffer, self)


@dataclass
class URDFModuleFixture:
    """A helper for launching a urdf module for use as a pytest fixture"""

    parameters: URDFModuleNodeFactory.Parameters
    """Useful for interrogating what parameters the fixture is using in a test"""

    @classmethod
    def set_up(
        cls,
        urdf_factory_params: URDFModuleNodeFactory.Parameters,
        static_transforms: list[tuple[str, str]] | None = None,
        retries: int = 3,
    ) -> Generator["URDFModuleFixture", None, None]:
        """Sets up the necessary nodes for test interaction with a URDF.

        :param urdf_factory_params: The parameters for the URDFModuleNodeFactory
        :param static_transforms: Static transforms to apply, as a list of
            (parent, child) frames. All transforms will be identity transforms.
            This feature is used to "glue" parts of this URDF to other existing
            TF's in the system.
        :param retries: The number of retries left. Default is 3.
        :yields: The module with all nodes started
        :raises URDFFixtureSetupFailed: If the setup fails after all retries
        """

        # Create various robot_state_publishers and joint_state_publishers
        urdf_module_factory = URDFModuleNodeFactory(parameters=urdf_factory_params)
        urdf_launch_nodes = urdf_module_factory.create_nodes()

        # Publish static transforms as configured by the URDF module
        static_transforms = static_transforms or []
        transform_node_generator = set_up_node(
            _TransformBroadcasterNode,
            namespace=urdf_factory_params.namespace,
            node_name="transform_broadcaster",
            multi_threaded=True,
        )

        # Spin up the joint & robot state publishers, and validate that the frames are
        # available in the TF tree. If they are not, retry up to 3 times to make up for
        # the flakiness of the robot_state_publisher and joint_state_publisher.
        urdf_nodes_generator = set_up_external_nodes_from_launchnode(urdf_launch_nodes)
        expected_frames = list(
            urdf_module_factory.urdf_constants.frames._asdict().values()
        ) + list(chain.from_iterable(static_transforms))

        try:
            with (
                _generator_context_manager(transform_node_generator) as transform_node,
                _generator_context_manager(urdf_nodes_generator),
            ):
                transform_node.add_static_transforms(static_transforms)

                # Wait until every frame in the URDFConstants and the static_transforms
                # has been published in the TF tree
                transform_node.wait_for_frames(expected_frames)

                yield URDFModuleFixture(urdf_factory_params)
        except URDFFixtureSetupFailed:
            if retries == 0:
                raise
            yield from cls.set_up(
                urdf_factory_params=urdf_factory_params,
                static_transforms=static_transforms,
                retries=retries - 1,
            )


class _TransformBroadcasterNode(HelpfulNode):
    """A basic node to fulfill some of the TF publishing and listening needs of the
    URDFModuleFixture.

    It handles:
    - Publishing basic static transforms
    - Listening to the TF buffer
    """

    def __init__(self, **kwargs: Any):
        super().__init__("transform_broadcaster", **kwargs)
        self.tf_buffer = Buffer(node=self)
        self._listener = TransformListener(self.tf_buffer, self)

    def wait_for_frames(self, frames: list[str]) -> None:
        """Waits until all of the frames can be accessed from every other frame
        and connected in the TF Tree
        :param frames: the list of frames to wait for
        :raises URDFFixtureSetupFailed: If the frames are not transformable
        """
        for a_frame in frames:
            for b_frame in frames:
                try:
                    self.tf_buffer.lookup_transform(
                        a_frame, b_frame, Time(), Duration(seconds=10)
                    )
                except (tf2_py.TransformException, tf2_py.ConnectivityException) as e:
                    raise URDFFixtureSetupFailed(
                        f"Could not find a transform between {a_frame} and {b_frame}. "
                        "Maybe you need to set the static_transforms parameter in the "
                        "URDFModuleFixture.set_up call, or maybe this is a flaky "
                        "test failure."
                    ) from e

    def add_static_transforms(self, static_transforms: list[tuple[str, str]]) -> None:
        for parent, child in static_transforms:
            ConstantStaticTransformBroadcaster(
                self,
                TransformStamped(header=Header(frame_id=parent), child_frame_id=child),
                publish_seconds=0.1,
            )


T = TypeVar("T")


@contextmanager
def _generator_context_manager(
    generator: Generator[T, None, None],
) -> Generator[T, None, None]:
    """Wraps the generator in a context manager, so the generator is always emptied"""
    try:
        yield next(generator)
    finally:
        tuple(generator)
