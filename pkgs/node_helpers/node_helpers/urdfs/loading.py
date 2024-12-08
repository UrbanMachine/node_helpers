from pathlib import Path
from typing import cast
from xml.etree import ElementTree

from ament_index_python import get_package_share_directory

NAMESPACE_FMT = "{namespace}.{name}"
_JOINT_TAG = "joint"
_LINK_TAG = "link"
_PARENT_TAG = "parent"
_CHILD_TAG = "child"
_NAME_KEY = "name"
_LINK_KEY = "link"
_MIMIC_TAG = "mimic"


def load_urdf(package: str, relative_urdf_path: Path | str) -> str:
    """Load a URDF as a string"""

    relative_urdf_path = Path(relative_urdf_path)
    package_root = get_package_share_directory(package)
    urdf_file = Path(package_root, relative_urdf_path)

    if not urdf_file.is_file():
        raise FileNotFoundError(f"URDF file '{urdf_file!s}' not found")
    return urdf_file.read_text()


def fix_urdf_paths(package: str, relative_urdf_path: Path | str) -> str:
    """Load a urdf and fix paths within the file to be relative to the package base.

    This assumes that all STLs and other resources required by the urdf are relative to
    the urdf file. For example, if the urdf file is in package/cool-dir/robot.urdf,
    then all the the STL's should also be found somewhere within cool-dir or in childs
    of that directory.

    Some tools like onshape-urdf-exporter don't include the package name in the stl
    paths within the robot.urdf file. This function fixes that by prepending the
    package name to the stl paths. If it's detected to already be there, it's left
    alone.

    :param package: The name of the package
    :param relative_urdf_path: The path to the urdf relative to the package directory
    :returns: The urdf text, with corrected paths.
    """
    relative_urdf_path = Path(relative_urdf_path)
    urdf_str = load_urdf(package, relative_urdf_path)

    if f"package://{package}" in urdf_str:
        # The urdf already has the package name in the paths, so we don't need to fix it
        return urdf_str

    urdf_str = urdf_str.replace(
        "package://", f"package://{package}/{relative_urdf_path.parent}/"
    )
    return urdf_str


def _assert_attributes_exist(
    urdf_strs: list[str], find_names: list[str], tag: str
) -> None:
    """Raise assertion errors if the names of a particular tag aren't specified in the
    URDF.

    :param urdf_strs: The list of URDF files, loaded as strings
    :param find_names: The list of joint names to validate
    :param tag: The XML tag to look for names under. For example, 'joint' or 'link'
    :raises ValueError: If the joint names don't exist
    """
    if not len(find_names):
        raise ValueError(f"There must be at least one {tag} name in order to validate!")

    urdf_tag_names = [_extract_tags(u, tag) for u in urdf_strs]

    seen_names = set()
    for name_set in urdf_tag_names:
        for name in name_set:
            if name in seen_names:
                raise ValueError(
                    f"The {tag} '{name}' has been seen in another URDF! When loading "
                    f"multiple URDFs with colliding names, make sure to apply "
                    f"namespaces to each."
                )

            if name in find_names:
                seen_names.add(name)

    if seen_names != set(find_names):
        raise ValueError(
            f"The following {tag} names were expected but were missing: "
            f"{set(find_names) - seen_names}"
        )


def _extract_tags(urdf_str: str, tag: str) -> list[str]:
    """Extract all unique names of a specific tag type from a URDF

    :param urdf_str: The URDF to parse
    :param tag: The tag to extract names of
    :return: The list of names found for that tag
    """
    urdf = ElementTree.fromstring(urdf_str)
    names = {cast(str, node.get(_NAME_KEY)) for node in urdf.iter(tag)}
    return list(names)


def assert_joint_names_exist(urdf_strs: list[str], names: list[str]) -> None:
    return _assert_attributes_exist(urdf_strs, names, _JOINT_TAG)


def assert_link_names_exist(urdf_strs: list[str], names: list[str]) -> None:
    return _assert_attributes_exist(urdf_strs, names, _LINK_TAG)


def prepend_namespace(urdf_str: str, namespace: str) -> str:
    """Apply a namespace to ever link and joint name, to make them unique
    :param urdf_str: A urdf laoded as text
    :param namespace: The 'namespace' to prepend link and joint names with
    :returns: The urdf text, with link and joint names prepended with the namespace
    """

    urdf = ElementTree.fromstring(urdf_str)

    # Rename all links and joints
    for node in urdf.iter():
        if node.tag in [_JOINT_TAG, _LINK_TAG]:
            node.set(
                "name",
                NAMESPACE_FMT.format(namespace=namespace, name=node.get(_NAME_KEY)),
            )
        elif (
            node.tag in [_PARENT_TAG, _CHILD_TAG] and "link" in node.keys()  # noqa: SIM118
        ):
            node.set(
                "link",
                NAMESPACE_FMT.format(namespace=namespace, name=node.get(_LINK_KEY)),
            )
        elif node.tag == _MIMIC_TAG:
            node.set(
                "joint",
                NAMESPACE_FMT.format(namespace=namespace, name=node.get(_NAME_KEY)),
            )
    return ElementTree.tostring(urdf, encoding="unicode")
