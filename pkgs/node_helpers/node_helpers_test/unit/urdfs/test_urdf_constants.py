from typing import NamedTuple

import pytest
from node_helpers.parameters import (
    DuplicateRegistrationError,
    UnregisteredChoosableError,
)
from node_helpers.urdfs import URDFConstants, URDFValidationError
from node_helpers_test.resources import GENERIC_URDF


class GenericURDFFrames(NamedTuple):
    BASE_LINK: str = "base_link"
    SOME_FRAME: str = "shuttle1"


class GenericURDFJoints(NamedTuple):
    JOINT_1: str = "shuttle1-joint"
    JOINT_2: str = "clamp1-joint"


def test_with_namespace() -> None:
    """
    Validate that *.with_namespace() returns new copies with the namespace prepended
    """
    constants: URDFConstants[GenericURDFJoints, GenericURDFFrames] = URDFConstants(
        from_package="node_helpers",
        registration_name="generic_urdf",
        urdf_paths=[(None, GENERIC_URDF)],
        joints=GenericURDFJoints(),
        frames=GenericURDFFrames(),
    )
    namespaced = constants.with_namespace("/bingus")
    assert constants.registration_name == "generic_urdf"
    assert namespaced.registration_name == "generic_urdf"
    assert constants.get_registered_instance("generic_urdf") is constants
    assert constants is not namespaced
    assert constants.namespace is None
    assert namespaced.namespace == "bingus"
    assert constants._urdf_paths == namespaced._urdf_paths
    assert constants.joints is not namespaced.joints
    assert constants.frames is not namespaced.frames
    assert isinstance(namespaced.frames, GenericURDFFrames)
    assert isinstance(namespaced.joints, GenericURDFJoints)
    assert f"bingus.{GenericURDFJoints().JOINT_2}" == namespaced.joints.JOINT_2
    assert f"bingus.{GenericURDFFrames().SOME_FRAME}" == namespaced.frames.SOME_FRAME


def test_multi_urdf_prefixes() -> None:
    """Test when each urdf has a prefix"""

    class MixedPrefixFrames(NamedTuple):
        BASE_LINK: str = "bingus.base_link"
        SOME_FRAME: str = "cool-prefix.shuttle1"

    class MixedPrefixJoints(NamedTuple):
        JOINT_1: str = "bingus.shuttle1-joint"
        JOINT_2: str = "cool-prefix.clamp1-joint"

    # Happy path! No failures here, presumably
    URDFConstants(
        from_package="node_helpers",
        registration_name="",
        urdf_paths=[("bingus", GENERIC_URDF), ("cool-prefix", GENERIC_URDF)],
        joints=MixedPrefixJoints(),
        frames=MixedPrefixFrames(),
    )

    # This should fail because the urdfs will have duplicate joint/frame names
    with pytest.raises(URDFValidationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name="",
            urdf_paths=[(None, GENERIC_URDF), (None, GENERIC_URDF)],
            joints=MixedPrefixJoints(),
            frames=MixedPrefixFrames(),
        )

    # This should fail because the 'cool-prefix' joints weren't found in any urdf
    with pytest.raises(URDFValidationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name="",
            urdf_paths=[(None, GENERIC_URDF), ("uncool-prefix", GENERIC_URDF)],
            joints=MixedPrefixJoints(),
            frames=MixedPrefixFrames(),
        )


def test_duplicate_urdf_files_require_prefixes() -> None:
    """Test that when two identical URDF files are loaded, the system requires a
    prefix on at least one of them."""

    with pytest.raises(URDFValidationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name="",
            urdf_paths=[(None, GENERIC_URDF), (None, GENERIC_URDF)],
            joints=GenericURDFJoints(),
            frames=GenericURDFFrames(),
        )
    with pytest.raises(URDFValidationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name="",
            urdf_paths=[("same-prefix", GENERIC_URDF), ("same-prefix", GENERIC_URDF)],
            joints=GenericURDFJoints(),
            frames=GenericURDFFrames(),
        )

    class PrefixedURDFFrames(NamedTuple):
        BASE_LINK_1: str = "prefixed_on_1.base_link"
        SOME_FRAME_1: str = "prefixed_on_1.shuttle1"
        BASE_LINK_2: str = "prefixed_on_2.base_link"
        SOME_FRAME_2: str = "prefixed_on_2.shuttle1"

    class PrefixedURDFJoints(NamedTuple):
        JOINT_1_1: str = "prefixed_on_1.shuttle1-joint"
        JOINT_1_2: str = "prefixed_on_1.clamp1-joint"
        JOINT_2_1: str = "prefixed_on_2.shuttle1-joint"
        JOINT_2_2: str = "prefixed_on_2.clamp1-joint"

    # Both are prefixed, yay
    URDFConstants(
        from_package="node_helpers",
        registration_name="cool-test-constant-91238",
        urdf_paths=[("prefixed_on_1", GENERIC_URDF), ("prefixed_on_2", GENERIC_URDF)],
        joints=PrefixedURDFJoints(),
        frames=PrefixedURDFFrames(),
    )


def test_validate_fails_with_false_path() -> None:
    with pytest.raises(FileNotFoundError):
        URDFConstants(
            from_package="node_helpers",
            registration_name="",
            urdf_paths=[("namespace", "invalid/path")],
            joints=GenericURDFJoints(),
            frames=GenericURDFFrames(),
        )


def test_nonexistent_frames_and_joints() -> None:
    """Test that joints and frames are cross-referenced with their parent urdf
    to ensure they exist
    """

    class BadFrames(NamedTuple):
        BAD_FRAME: str = "this frame does not exist!"

    class BadJoints(NamedTuple):
        BAD_JOINT: str = "This doesn't exist!"

    # Test with bad joints
    with pytest.raises(URDFValidationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name="",
            urdf_paths=[(None, GENERIC_URDF)],
            joints=BadJoints(),
            frames=GenericURDFFrames(),
        )

    # Test with bad frames
    with pytest.raises(URDFValidationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name="",
            urdf_paths=[(None, GENERIC_URDF)],
            joints=GenericURDFJoints(),
            frames=BadFrames(),
        )


def test_duplicate_frame_names_and_joints() -> None:
    """Validate that duplicate joint names are found and cause failures"""

    class DuplicateFrames(NamedTuple):
        FRAME_1: str = GenericURDFFrames().SOME_FRAME
        FRAME_2: str = GenericURDFFrames().SOME_FRAME

    class DuplicateJoints(NamedTuple):
        JOINT_1: str = GenericURDFJoints().JOINT_1
        JOINT_2: str = GenericURDFJoints().JOINT_1

    # Test with duplicate joints
    with pytest.raises(URDFValidationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name="",
            urdf_paths=[(None, GENERIC_URDF)],
            joints=DuplicateJoints(),
            frames=GenericURDFFrames(),
        )

    # Test with duplicate frames
    with pytest.raises(URDFValidationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name="",
            urdf_paths=[(None, GENERIC_URDF)],
            joints=GenericURDFJoints(),
            frames=DuplicateFrames(),
        )


def test_applying_namespace_twice_fails() -> None:
    not_namespaced: URDFConstants[GenericURDFJoints, GenericURDFFrames] = URDFConstants(
        from_package="node_helpers",
        registration_name="cool-test-constant-142",
        urdf_paths=[(None, GENERIC_URDF)],
        joints=GenericURDFJoints(),
        frames=GenericURDFFrames(),
    )
    namespaced = not_namespaced.with_namespace("cool_stuff")
    with pytest.raises(ValueError):
        namespaced.with_namespace("whoa_namespaceception")


def test_registration() -> None:
    """Test that URDF constants are registered when expected"""
    registered_name = "registered-urdf-constant"

    # This should fail validation, and not be registered
    class DuplicateJoints(NamedTuple):
        JOINT_1: str = GenericURDFJoints().JOINT_1
        JOINT_2: str = GenericURDFJoints().JOINT_1

    with pytest.raises(URDFValidationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name=registered_name,
            urdf_paths=[(None, GENERIC_URDF)],
            joints=DuplicateJoints(),
            frames=GenericURDFFrames(),
        )

    # Ensure it was not registered
    with pytest.raises(UnregisteredChoosableError):
        URDFConstants.get_registered_instance(registered_name)

    # Because validation failed, this should not be registered
    with pytest.raises(UnregisteredChoosableError):
        URDFConstants.get_registered_instance(registered_name)

    # This should work, and be registered
    registered = URDFConstants(
        from_package="node_helpers",
        registration_name=registered_name,
        urdf_paths=[(None, GENERIC_URDF)],
        joints=GenericURDFJoints(),
        frames=GenericURDFFrames(),
    )

    # Validate it's registered
    assert URDFConstants.get_registered_instance(registered_name) is registered

    # Making instances by using 'with_namespace' should pass, but not replace the OG
    new_instance = registered.with_namespace("blah")
    assert URDFConstants.get_registered_instance(registered_name) is not new_instance

    # This should fail because the name is already registered
    with pytest.raises(DuplicateRegistrationError):
        URDFConstants(
            from_package="node_helpers",
            registration_name=registered_name,
            urdf_paths=[(None, GENERIC_URDF)],
            joints=GenericURDFJoints(),
            frames=GenericURDFFrames(),
        )
