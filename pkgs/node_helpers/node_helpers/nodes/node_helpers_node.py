"""This is a total example node to show off some simple node_helpers features. It's not
meant to be a comprehensive example, but rather a simple one to show off some of the
features of the node_helpers package.
"""

from typing import Any

from pydantic import BaseModel
from rclpy.qos import qos_profile_services_default
from sensor_msgs.msg import JointState

from node_helpers.example_urdf import ForkliftURDF
from node_helpers.nodes import HelpfulNode
from node_helpers.spinning import create_spin_function


class ExampleNode(HelpfulNode):
    class Parameters(BaseModel):
        # Define your ROS parameters here
        forklift_speed: float  # m/s
        forklift_max_extent: float

    def __init__(self, **kwargs: Any):
        super().__init__("ExampleNode", **kwargs)
        # Load parameters from the ROS parameter server
        self.params = self.declare_from_pydantic_model(self.Parameters, "root_config")
        self.urdf = ForkliftURDF.with_namespace(self.get_namespace())

        # Track the forks position and direction, so we can move them up and down
        self.forklift_position = 0.0
        self.forklift_direction = 1

        # Create publishers
        self.joint_state_publisher = self.create_publisher(
            JointState, "desired_joint_states", qos_profile_services_default
        )

        # Create a timer to publish joint values
        self._publish_hz = 20
        self.create_timer(1 / self._publish_hz, self.on_publish_joints)

    def on_publish_joints(self) -> None:
        if self.forklift_position >= self.params.forklift_max_extent:
            self.forklift_direction = -1
        elif self.forklift_position <= 0:
            self.forklift_direction = 1

        self.forklift_position += (
            self.forklift_direction * self.params.forklift_speed / self._publish_hz
        )

        joint_positions = {self.urdf.joints.FORKS: self.forklift_position}

        self.joint_state_publisher.publish(
            JointState(
                name=list(joint_positions.keys()),
                position=list(joint_positions.values()),
            )
        )


main = create_spin_function(ExampleNode)
