from typing import NamedTuple

from node_helpers.urdfs import URDFConstants


class ForkliftJoints(NamedTuple):
    FORKS: str = "forks"
    FORKS_PARENT_DATUM: str = "forks_parent_datum"


class ForkliftFrames(NamedTuple):
    BASE_LINK: str = "forklift_body"

    # Joint tracking
    FORKS_ORIGIN: str = "forks_origin"
    FORKS: str = "forks"


ForkliftURDF = URDFConstants[ForkliftJoints, ForkliftFrames](
    from_package="node_helpers",
    registration_name="test_forklift",
    urdf_paths=[(None, "sample_urdfs/forklift/robot.urdf")],
    joints=ForkliftJoints(),
    frames=ForkliftFrames(),
)
