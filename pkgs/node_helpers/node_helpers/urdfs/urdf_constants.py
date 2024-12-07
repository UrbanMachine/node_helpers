import logging
from collections.abc import Sequence
from pathlib import Path
from typing import Generic, NamedTuple, TypeVar

from node_helpers.parameters import Choosable, DuplicateRegistrationError

from . import loading as urdf_helpers

JOINTS = TypeVar("JOINTS", bound=NamedTuple)
FRAMES = TypeVar("FRAMES", bound=NamedTuple)
EITHER = TypeVar("EITHER", bound=NamedTuple)


class URDFValidationError(Exception):
    """Occurs during validation of a URDF"""


class URDFConstants(Choosable, Generic[JOINTS, FRAMES]):
    def __init__(
        self,
        from_package: str,
        registration_name: str,
        urdf_paths: Sequence[tuple[str | None, str | Path]],
        joints: JOINTS,
        frames: FRAMES,
        prepend_namespace: str | None = None,
    ):
        """
        :param from_package: The package to load the URDFs from
        :param registration_name: The name to use when registering this URDF with the
            global registry. Using this name you can access this URDF from anywhere,
            using urdf.get_registered_instance(name).
            Instances created using 'with_namespace' are not automatically registered.
        :param urdf_paths: A list of tuples of form [("prepend-name", "urdf_path"), ...]
            The first element in the tuple can be None or a string, and represents a
            name to tack on to the start of every joint/link in the urdf. This will
            allow for tacking together the same URDF multiple times into a larger
            assembly. All urdf paths are relative to the urdf package root.
        :param joints: The Joints model
        :param frames: The Frames model
        :param prepend_namespace: A namespace, if any, to prepend to joints and frames

        :raises DuplicateRegistrationError: If you use the same name on multiple URDFs
        """

        self.registration_name = registration_name
        self.namespace = prepend_namespace
        self.from_package = from_package
        self.frames = self._prepend_namespace(frames, self.namespace)
        self.joints = self._prepend_namespace(joints, self.namespace)

        self._urdf_paths = [(namespace, Path(path)) for namespace, path in urdf_paths]

        self.validate()

        # Register this instance with the global registry, so that it can be accessed
        # via URDFConstant.get_registered_instance(name)
        try:
            self.register_instance(self.registration_name)
        except DuplicateRegistrationError:
            if prepend_namespace is None:
                # This means that a URDF with this name was already registered, but it
                # wasn't simply because someone instantiated a new URDF using
                # urdf.with_namespace(...)!
                raise
            logging.info(
                f"URDFConstant with name {self.registration_name} was already "
                f"registered. This is fine, because the URDF had a namespace applied."
            )

    def __len__(self) -> int:
        return len(self._urdf_paths)

    @staticmethod
    def _prepend_namespace(model: EITHER, namespace: str | None) -> EITHER:
        """Create a version of this model with a namespace applied to each attribute"""

        if namespace is None:
            return model
        namespace = namespace.replace("/", "")

        new_attributes: dict[str, str] = {}
        for model_attr, attr_constant in model._asdict().items():
            namespaced_constant = urdf_helpers.NAMESPACE_FMT.format(
                namespace=namespace, name=attr_constant
            )
            new_attributes[model_attr] = namespaced_constant

        # Return a newly generated namedtuple
        return type(model)(**new_attributes)  # type: ignore

    def validate(self) -> None:
        urdf_strs = self.load_urdfs()

        joint_names = list(self.joints)
        frame_names = list(self.frames)

        # Validate that if two URDFs with the same path got loaded, that they all have a
        # prefix associated with them.
        for path_and_prefix in self._urdf_paths:
            duplicates = [
                other for other in self._urdf_paths if other == path_and_prefix
            ]
            if len(duplicates) > 1:
                raise URDFValidationError(
                    "When loading multiple of the same URDF file in one Constants, you "
                    "must apply a prefix to at least one of them."
                )

        # Validate joint and frame names exist in the URDF
        try:
            if len(joint_names):
                urdf_helpers.assert_joint_names_exist(urdf_strs, joint_names)

            if len(frame_names):
                urdf_helpers.assert_link_names_exist(urdf_strs, frame_names)

        except ValueError as e:
            raise URDFValidationError(str(e)) from e

        # Validate there are no duplicate frame or joint names
        if not (
            len(frame_names) == len(set(frame_names))
            and len(joint_names) == len(set(joint_names))
        ):
            raise URDFValidationError(
                f"There can be no duplicate frame or joint names! "
                f"{self.frames=} {self.joints=}"
            )

    def load_urdfs(self) -> list[str]:
        """Load each urdf with a prepended namespace, if configured.
        URDFs will also have their respective configured prefixes
        (configured in the constructor) applied.

        :return: A list of URDFs loaded as strings
        """

        urdfs = []
        for custom_namespace, urdf_path in self._urdf_paths:
            urdf_str = urdf_helpers.fix_urdf_paths(self.from_package, urdf_path)

            if custom_namespace is not None:
                urdf_str = urdf_helpers.prepend_namespace(urdf_str, custom_namespace)
            if self.namespace is not None:
                urdf_str = urdf_helpers.prepend_namespace(urdf_str, self.namespace)
            urdfs.append(urdf_str)
        return urdfs

    def with_namespace(self, namespace: str) -> "URDFConstants[JOINTS, FRAMES]":
        """Returns URDFConstants with the namespace prepended to frames and joints"""
        if self.namespace is not None:
            raise ValueError(
                "You can't add a namespace to a URDFConstants that already "
                "had a namespace applied"
            )

        return URDFConstants(
            from_package=self.from_package,
            registration_name=self.registration_name,
            urdf_paths=self._urdf_paths,
            joints=self.joints,
            frames=self.frames,
            prepend_namespace=namespace.replace("/", ""),
        )
