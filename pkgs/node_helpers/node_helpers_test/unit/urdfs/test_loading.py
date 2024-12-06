import pytest
from node_helpers.urdfs import loading as urdf_helpers
from node_helpers.urdfs.loading import NAMESPACE_FMT
from node_helpers_test.resources import GENERIC_URDF

EXPECTED_JOINT_NAMES = ["shuttle1-joint", "clamp1-joint"]
EXPECTED_LINK_NAMES = ["base_link", "shuttle1", "clamp1"]


def test_fix_urdf_paths_makes_path_replacements() -> None:
    package_name = "node_helpers"
    expected_pattern = f"{package_name}/{GENERIC_URDF.parent}/"

    modified_urdf = urdf_helpers.fix_urdf_paths(
        package=package_name, relative_urdf_path=GENERIC_URDF
    )
    original_urdf = GENERIC_URDF.read_text()
    assert modified_urdf != original_urdf

    n_expected_changes = original_urdf.count("package://")
    assert n_expected_changes == modified_urdf.count(expected_pattern)
    assert original_urdf.count(expected_pattern) == 0


def test_assert_joint_names_exist() -> None:
    urdf_text = GENERIC_URDF.read_text()

    # Raises error when no names provided
    with pytest.raises(ValueError):
        urdf_helpers.assert_joint_names_exist([urdf_text], [])

    # Raises an error when invalid names provided
    with pytest.raises(ValueError):
        urdf_helpers.assert_joint_names_exist(
            [urdf_text], [*EXPECTED_JOINT_NAMES, "nonexistent-joint"]
        )

    # No assertions raised when all joint names are valid
    urdf_helpers.assert_joint_names_exist([urdf_text], EXPECTED_JOINT_NAMES)


def test_assert_attributes_exist_fails_on_duplicate_attributes_across_urdfs() -> None:
    """The basic contract with multi-urdf loading is that each urdf has unique frame
    or joint names; at least, the ones that are being asserted. This test validates
    that check is being done as expected."""

    urdf_text = GENERIC_URDF.read_text()
    urdf_text_namespaced = urdf_helpers.prepend_namespace(
        GENERIC_URDF.read_text(), "cool"
    )

    _expected_links_namespaced = [
        NAMESPACE_FMT.format(namespace="cool", name=n) for n in EXPECTED_LINK_NAMES
    ]

    # Raises error when two urdfs have the same joint name
    with pytest.raises(ValueError):
        urdf_helpers.assert_link_names_exist(
            [urdf_text, urdf_text], EXPECTED_LINK_NAMES
        )

    # This should fail because the joint names don't actually exist
    with pytest.raises(ValueError):
        urdf_helpers.assert_link_names_exist(
            [urdf_text_namespaced, urdf_text_namespaced], EXPECTED_LINK_NAMES
        )

    # Test happy path with just one urdf file
    urdf_helpers.assert_link_names_exist(
        [urdf_text_namespaced], _expected_links_namespaced
    )
    urdf_helpers.assert_link_names_exist([urdf_text], EXPECTED_LINK_NAMES)

    # Test happy paths where the two urdfs are namespaced, and the joints all exist
    urdf_helpers.assert_link_names_exist(
        [urdf_text, urdf_text_namespaced],
        [*_expected_links_namespaced, *EXPECTED_LINK_NAMES],
    )
    urdf_helpers.assert_link_names_exist(
        [urdf_text, urdf_text_namespaced], EXPECTED_LINK_NAMES
    )
    urdf_helpers.assert_link_names_exist(
        [urdf_text, urdf_text_namespaced], _expected_links_namespaced
    )


def test_assert_link_names_exist() -> None:
    urdf_text = GENERIC_URDF.read_text()

    # Raises error when no names provided
    with pytest.raises(ValueError):
        urdf_helpers.assert_link_names_exist([urdf_text], [])

    # Raises an error when invalid names provided
    with pytest.raises(ValueError):
        urdf_helpers.assert_link_names_exist(
            [urdf_text], [*EXPECTED_LINK_NAMES, "nonexistent-link"]
        )

    # No assertions raised when all link names are valid
    urdf_helpers.assert_link_names_exist([urdf_text], EXPECTED_LINK_NAMES)


def test_prepend_namespace() -> None:
    urdf_text: str = GENERIC_URDF.read_text()
    namespace = "cool_namespace"
    modified = urdf_helpers.prepend_namespace(urdf_text, namespace=namespace)

    for changes in (EXPECTED_JOINT_NAMES, EXPECTED_LINK_NAMES):
        expected_changes = list(map(urdf_text.count, changes))
        actual_changes = list(
            map(
                modified.count,
                [
                    urdf_helpers.NAMESPACE_FMT.format(namespace=namespace, name=j)
                    for j in changes
                ],
            )
        )
        assert len(expected_changes) == len(actual_changes)
