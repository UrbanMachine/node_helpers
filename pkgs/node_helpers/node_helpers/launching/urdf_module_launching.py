from functools import reduce
from operator import iconcat
from typing import Any

from launch_ros.actions import Node
from pydantic import BaseModel

from node_helpers.urdfs.urdf_constants import URDFConstants


class URDFModuleNodeFactory:
    """A helper object for creating nodes for a urdf module.

    This class takes a URDFConstant and a namespace, and then for each child URDF will:
    1) Spin up a robot state publisher under the namespace '/{namespace}/urdf_{idx}'
    2) Spin up a joint state publisher under the above namespace.

    If there are 3 URDFs in the URDFConstant, there will be 6 nodes total launched.
    Potential future optimizations include only spinning up `joint_state_publishers` if
    there happen to be any joints in the URDF being spun up.
    """

    class Parameters(BaseModel):
        namespace: str
        """The namespace under which under which joint state publishers and robot state
        publishers will live, a la /{namespace}/urdf_# """

        urdf_constant_name: str
        """The chosen URDFConstant.registration_name to spin up. In configuration, you
        can reference these as strings, using the name attribute to load a specific
        instance of a URDFConstant."""

        apply_namespace_to_urdf: bool = True
        """If True, the node namespace will be prepended to the URDF frames. This is
        the behaviour used by hardware modules. Set this to False if your URDF is not
        part of a hardware module."""

    def __init__(self, parameters: Parameters):
        self._params = parameters

        # Create the URDFConstant, with a namespace optionally prepended
        base_urdf_constants = URDFConstants[Any, Any].get_registered_instance(
            self._params.urdf_constant_name
        )
        self.urdf_constants = (
            base_urdf_constants.with_namespace(self._params.namespace)
            if self._params.apply_namespace_to_urdf
            else base_urdf_constants
        )

    def create_nodes(self) -> list[Node]:
        """Create the nodes required to load and visualize each specified urdf path"""
        urdf_strs = self.urdf_constants.load_urdfs()

        urdf_nodes = [
            [
                self.create_robot_state_publisher(
                    namespace=self._params.namespace,
                    urdf_index=urdf_index,
                    urdf_str=urdf_str,
                ),
                self.create_joint_state_publisher(
                    namespace=self._params.namespace,
                    urdf_index=urdf_index,
                ),
            ]
            for urdf_index, urdf_str in enumerate(urdf_strs)
        ]
        return reduce(iconcat, urdf_nodes, [])

    @staticmethod
    def create_joint_state_publisher(namespace: str, urdf_index: int) -> Node:
        return Node(
            package="joint_state_publisher",
            namespace=URDFModuleNodeFactory.urdf_namespace(namespace, urdf_index),
            executable="joint_state_publisher",
            parameters=[
                {
                    "source_list": [
                        f"/{namespace}/desired_joint_states",
                    ]
                }
            ],
        )

    @staticmethod
    def create_robot_state_publisher(
        namespace: str,
        urdf_index: int,
        urdf_str: str,
    ) -> Node:
        """Create a robot state publisher using the hardware module standards
        :param namespace: The namespace under which to create the urdf namespace
        :param urdf_index: The index of this urdf within the parent namespace
        :param urdf_str: The urdf as a string to pass to the robot_state_publisher
        :return: The robot state publisher node.
        """

        return Node(
            package="robot_state_publisher",
            namespace=URDFModuleNodeFactory.urdf_namespace(namespace, urdf_index),
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": urdf_str},
            ],
        )

    @property
    def urdf_namespaces(self) -> list[str]:
        """Returns the namespaces under which URDFs are stored, for rviz remapping."""
        return [
            self.urdf_namespace(self._params.namespace, urdf_id)
            for urdf_id in range(len(self.urdf_constants))
        ]

    @staticmethod
    def urdf_namespace(namespace: str, urdf_index: int) -> str:
        """A helper for creating the namespace for a given urdf in a module

        :param namespace: The parent namespace that will own one or more URDFs
        :param urdf_index: The index of this particular urdf
        :return: The formatted namespace string
        """
        return f"{namespace}/urdf_{urdf_index}"
