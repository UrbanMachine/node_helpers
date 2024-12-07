from collections.abc import Generator
from typing import cast

import pytest
from node_helpers.launching import URDFModuleNodeFactory
from node_helpers.testing import (
    TFClient,
    URDFModuleFixture,
    set_up_node,
    validate_coincident_transforms,
)

from .example_urdf_constants import ForkliftURDF

_NAMESPACED_CONSTANTS = ForkliftURDF.with_namespace("test_fixture")
_FRAMES = _NAMESPACED_CONSTANTS.frames
_JOINTS = _NAMESPACED_CONSTANTS.joints


@pytest.fixture()
def tf_client() -> Generator[TFClient, None, None]:
    yield from set_up_node(TFClient, "cool_namespace", "tf_node")


@pytest.fixture()
def urdf_module() -> Generator[URDFModuleFixture, None, None]:
    yield from URDFModuleFixture.set_up(
        URDFModuleNodeFactory.Parameters(
            namespace=cast(str, _NAMESPACED_CONSTANTS.namespace),
            urdf_constant_name=ForkliftURDF.registration_name,
        ),
        static_transforms=[],
    )


@pytest.mark.parametrize(
    ("joint_origin", "joint_current", "same_position", "same_orientation"),
    ((_FRAMES.FORKS, _FRAMES.FORKS_ORIGIN, True, True),),
)
def test_coincident_transforms(
    urdf_module: URDFModuleFixture,
    tf_client: TFClient,
    joint_origin: str,
    joint_current: str,
    same_position: bool,
    same_orientation: bool,
) -> None:
    """This test validates that the URDF is correctly formulated, such that the joint
    `joint_current` is coincident with the joint `joint_origin` when the robot is in its
    0 position.
    """
    validate_coincident_transforms(
        urdf_module,
        tf_client,
        joint_origin,
        joint_current,
        same_position,
        same_orientation,
    )
