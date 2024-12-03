import tempfile
from copy import deepcopy
from pathlib import Path
from typing import Any, Generic, TypeVar

import yaml
from pydantic import BaseModel
from rclpy.parameter import Parameter

MetaParameters = TypeVar("MetaParameters", bound=BaseModel)


class Namespace:
    """A representation of ROS's period-separated namespacing system. This can be used
    to represent a fully qualified node name, like 'manipulator.mock_manipulator', or a
    nested parameter name, like 'steppers.x.max_pos'.
    """

    def __init__(self, value: list[str]) -> None:
        self.items = value

    @staticmethod
    def from_string(value: str) -> "Namespace":
        return Namespace(value.split("."))

    def __str__(self) -> str:
        return ".".join(self.items)

    def __repr__(self) -> str:
        return f"Namespace({self!s})"

    def __add__(self, other: "Namespace") -> "Namespace":
        return Namespace(self.items + other.items)

    def with_item(self, item: str) -> "Namespace":
        return Namespace(self.items + [item])

    def __hash__(self) -> int:
        return hash(tuple(self.items))

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Namespace):
            return self.items == other.items
        raise NotImplementedError


class ParameterLoader(Generic[MetaParameters]):
    """This class facilitates a workflow for ROS development that expects there to be
    two layers of arguments/parameters/configuration that is typically needed for a
    project:

        1) The meta parameters. These are all parameters needed for the launch file
           itself, and are not used to feed arguments into nodes themselves.
        2) The node parameters. These are actual ROS parameters that can be fed into
           individual nodes.

    The workflow includes loading one or more yaml files and combining them into a
    single file, allowing for a layered configuration approach in which common
    configuration is placed in a base parameters file, and 'override' files (which are
    not typically committed via git) can be created for different hardware environments
    or use cases.
    """

    def __init__(
        self,
        *,
        parameters_directory: Path,
        override_file: Path | None = None,
        meta_parameters_schema: type[MetaParameters] | None = None,
    ) -> None:
        """
        :param parameters_directory: A directory with one or more yaml files to try
            loading. All yaml files will be loaded in alphanumeric order by file name,
            and any colliding keys will override the previous entry.
        :param override_file: A file to load last, which will override any previous
            configuration.
        :param meta_parameters_schema: The pydantic schema with which to load the
            "meta_parameters" section of the configuration ros_parameters_file.
        :raises FileNotFoundError: If no yaml files are found in the provided directory
        """

        files_in_order = [*sorted(parameters_directory.glob("*.yaml"))]
        if len(files_in_order) == 0:
            raise FileNotFoundError(f"No yaml files found in '{parameters_directory}'!")

        if override_file:
            files_in_order.append(override_file)

        yaml_dict: dict[str, Any] = {}
        for file_ in filter(Path.is_file, files_in_order):
            yaml_dict_to_merge = yaml.full_load(file_.read_text())
            if yaml_dict_to_merge is not None:
                yaml_dict = self._merge_dictionaries(yaml_dict, yaml_dict_to_merge)

        # Load the meta parameters, if a schema was specified
        self._meta_parameters = None
        if meta_parameters_schema:
            self._meta_parameters = self._load_meta_params(
                yaml_dict, meta_parameters_schema
            )

        # Load the ros node parameters
        self.parameters: dict[Namespace, dict[Namespace, Any]] = {}
        self._load_file_params(yaml_dict)

        # Save the ros node parameters to a file, for easy parameter injection
        yaml_dict.pop(_META_PARAMETERS_KEY, None)
        self.ros_parameters_file = self._save_yaml(yaml_dict)
        """A path a yaml ros_parameters_file with the attributes combined, containing
        the result of merging all provided parameter files together, without the
        meta parameters.
        """

    @property
    def meta_parameters(self) -> MetaParameters:
        if self._meta_parameters is None:
            raise RuntimeError(
                "You cannot use meta_parameters if a schema was not passed in to the "
                "ParameterLoader!"
            )
        return self._meta_parameters

    def parameters_for_node(self, namespace: Namespace) -> list[Parameter]:
        """
        :param namespace: The namespace and name of the node, like
            "manipulator.manipulator_capcom"
        :return: The node's parameters, in a format that can be provided to a node's
            parameter_overrides argument
        """
        node_params = self.parameters.get(namespace, {})
        return [Parameter(name=str(k), value=v) for k, v in node_params.items()]

    @staticmethod
    def _merge_dictionaries(
        a: dict[str, Any], b: dict[str, Any], path: list[str] | None = None
    ) -> dict[str, Any]:
        """Merges dictionary b into a and returns the resulting dictionary."""
        a, b = deepcopy(a), deepcopy(b)

        if path is None:
            path = []
        for key in b:
            if key in a:
                if isinstance(a[key], dict) and isinstance(b[key], dict):
                    a[key] = ParameterLoader._merge_dictionaries(
                        a[key], b[key], path + [str(key)]
                    )
                elif a[key] != b[key]:
                    a[key] = b[key]
            else:
                a[key] = b[key]
        return a

    def _load_meta_params(
        self, data_yaml: dict[str, Any], schema: type[MetaParameters]
    ) -> MetaParameters:
        if not data_yaml.get(_META_PARAMETERS_KEY, False):
            raise ParameterLoadingError(
                "A schema was specified for meta parameters, but the key "
                f"'{_META_PARAMETERS_KEY}' was not found!"
            )
        meta_parameters_dict = data_yaml[_META_PARAMETERS_KEY]
        return schema.model_validate(meta_parameters_dict)

    def _load_file_params(self, data_yaml: dict[str, Any]) -> None:
        for key, fields in data_yaml.items():
            try:
                self._load_namespace_params(fields, Namespace([key]))
            except ParameterLoadingError as ex:
                raise ParameterLoadingError(f"In field {key}: {ex}") from ex

    def _load_namespace_params(
        self, data_yaml: dict[str, Any], node_name: Namespace
    ) -> None:
        for key, value in data_yaml.items():
            if isinstance(value, dict):
                if key == _PARAMETERS_KEY:
                    self._load_node_params(value, node_name, Namespace([]))
                else:
                    self._load_namespace_params(value, node_name.with_item(key))

    def _load_node_params(
        self, input_: dict[str, Any], node_name: Namespace, param_name: Namespace
    ) -> None:
        for key, value in input_.items():
            if isinstance(value, dict):
                self._load_node_params(value, node_name, param_name.with_item(key))
            else:
                if node_name not in self.parameters:
                    self.parameters[node_name] = {}
                self.parameters[node_name][param_name.with_item(key)] = value

    def _save_yaml(self, yaml_dict: dict[str, Any]) -> Path:
        # Unfortunately this is the only way to make the yaml writer not create weird
        # files full of yaml aliases that ROS can't parse.
        yaml.Dumper.ignore_aliases = lambda *args: True  # type: ignore
        _YAML_FILE.write_text(yaml.dump(yaml_dict))
        return _YAML_FILE


class ParameterLoadingError(Exception):
    """The parameters file has a formatting issue"""


FIELD_PLACEHOLDER = "<override this>"
"""The placeholder text put on parameters that need to be filled in manually"""

_PARAMETERS_KEY = "ros__parameters"
"""ROS's magic key that signals when node namespacing ends and parameters begin"""

_META_PARAMETERS_KEY = "meta_parameters"
"""The name of the node at the root of the ros_parameters_file that holds the meta
parameters for the launch file. """

_YAML_FILE = Path(tempfile.gettempdir()) / "parameters.yaml"
"""The temporary ros_parameters_file location to write the final merged
 ros_parameters_file to"""
