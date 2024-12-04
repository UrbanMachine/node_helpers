import contextlib
from collections.abc import Generator
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any, cast

import pytest
import yaml
from node_helpers.parameters import (
    FIELD_PLACEHOLDER,
    Namespace,
    ParameterLoader,
    loading,
)
from pydantic import BaseModel
from rclpy.parameter import Parameter


def test_namespace_with_item_copies() -> None:
    """Tests that Namespace.with_item creates an independent copy of its items"""
    namespace1 = Namespace(["manipulators", "bomb_defuser"])
    namespace2 = namespace1.with_item("red_wire")

    namespace1.items.append("blue_wire")

    assert namespace1.items == ["manipulators", "bomb_defuser", "blue_wire"]
    assert namespace2.items == ["manipulators", "bomb_defuser", "red_wire"]


def test_parameter_loader_base_loading() -> None:
    """Tests loading of a single parameter file"""
    with _parameter_directory(_BASE_PARAMETERS_YAML) as parameter_dir:
        loader = ParameterLoader(
            parameters_directory=parameter_dir,
            meta_parameters_schema=MetaParameters,
        )

    assert loader.parameters == _EXPECTED_PARAMETERS
    assert loader.meta_parameters == _EXPECTED_META_PARAMETERS


def test_parameter_loader_override_loading() -> None:
    """Tests that parameters and meta parameters are combined as expected when loading
    of base and override parameter file
    """
    with (
        _parameter_directory(_BASE_PARAMETERS_YAML) as parameter_dir,
        _parameter_file(_OVERRIDE_PARAMETERS_YAML) as override_path,
    ):
        loader = ParameterLoader(
            parameters_directory=parameter_dir,
            override_file=override_path,
            meta_parameters_schema=MetaParameters,
        )

    # Validate ros parameters
    expected_parameters = cast(
        dict[Namespace, dict[Namespace, Any]], deepcopy(_EXPECTED_PARAMETERS)
    )
    expected_parameters[Namespace(["nearby_theater"])][
        Namespace(["incident_report", "description"])
    ] = "How wrong I was! Friendship is a beautiful thing."
    expected_parameters[Namespace(["nearby_theater"])][
        Namespace(["lessons_learned"])
    ] = True

    assert loader.parameters == expected_parameters

    # Validate meta parameters
    expected_meta_parameters = deepcopy(_EXPECTED_META_PARAMETERS.model_dump())
    expected_meta_parameters["hangouts"] = ["detention"]
    assert loader.meta_parameters.model_dump() == expected_meta_parameters


def test_parameter_loader_file_saving() -> None:
    """Tests that saving the resulting canonical parameter file has the expected data.
    It also shouldn't include any meta parameters data."""
    with _parameter_directory(_BASE_PARAMETERS_YAML) as parameter_dir:
        loader: ParameterLoader[BaseModel] = ParameterLoader(
            parameters_directory=parameter_dir
        )

    loaded_text = yaml.full_load(loader.ros_parameters_file.read_text())
    base_parameters = yaml.full_load(_BASE_PARAMETERS_YAML)
    del base_parameters[loading._META_PARAMETERS_KEY]
    assert loaded_text == base_parameters


def test_parameter_loader_for_node() -> None:
    """Tests getting parameters for a single node from the loader"""
    with _parameter_directory(_BASE_PARAMETERS_YAML) as parameter_dir:
        loader: ParameterLoader[BaseModel] = ParameterLoader(
            parameters_directory=parameter_dir
        )

    params = loader.parameters_for_node(Namespace(["high_school_movie", "cool_kid"]))
    assert len(params) == 1
    assert params[0].name == "is_cool"
    assert params[0].type_ == Parameter.Type.BOOL
    assert params[0].value


def test_parameter_loader_loads_without_meta_parameters_schema_provided() -> None:
    """Test that not providing a meta parameters schema is okay, but you can't access
    the meta_parameters property without an error being raised."""
    with _parameter_directory(_BASE_PARAMETERS_YAML) as parameter_dir:
        loader: ParameterLoader[BaseModel] = ParameterLoader(
            parameters_directory=parameter_dir
        )

    # The class shouldn't let you load meta parameters when a schema wasn't initially
    # provided.
    with pytest.raises(RuntimeError):
        loader.meta_parameters  # noqa: B018
    assert loader._meta_parameters is None


def test_parameter_loader_invalid_meta_parameters() -> None:
    """Test that an exception is raised when a meta parameters schema is given but
    the meta parameters key isn't in the yaml."""
    with _parameter_directory(_NO_META_PARAMETERS_YAML) as parameter_dir:
        # This should work
        ParameterLoader(parameters_directory=parameter_dir)

        # This should fail
        with pytest.raises(loading.ParameterLoadingError):
            ParameterLoader(
                parameters_directory=parameter_dir,
                meta_parameters_schema=MetaParameters,
            )


def test_merge_dictionaries_deepcopies() -> None:
    """Make sure that merge_dictionary isn't editing in place."""
    val_1 = object()
    val_2 = object()
    val_2_replacement = object()
    val_3 = object()

    a = {"key_1": val_1, "key_2": val_2}
    b = {"key_2": val_2_replacement, "key_3": val_3}

    merged = ParameterLoader._merge_dictionaries(a, b)

    assert ["key_1", "key_2", "key_3"] == list(merged.keys())
    for val in merged.values():
        assert val not in (val_1, val_2, val_3, val_2_replacement)


@contextlib.contextmanager
def _parameter_file(value: str) -> Generator[Path, None, None]:
    with TemporaryDirectory() as temp_dir:
        temp_file = Path(temp_dir) / "some-params.yaml"
        with temp_file.open("w") as f:
            f.write(value)
            f.flush()

        yield Path(temp_file)


@contextlib.contextmanager
def _parameter_directory(value: str) -> Generator[Path, None, None]:
    with _parameter_file(value) as f:
        yield f.parent


class MetaParameters(BaseModel):
    school_name: str
    cool_kid_to_dork_ratio: float
    hangouts: list[str]


_BASE_PARAMETERS_YAML = """
meta_parameters:
    school_name: Rydell High School
    cool_kid_to_dork_ratio: 0.9
    hangouts: ["corner store", "under the bleachers"]

high_school_movie:
  cool_kid:
    ros__parameters:
      is_cool: true
  dork_kid:
    ros__parameters:
      catchphrase: This math stuff is easy

nearby_theater:
  ros__parameters:
    employees:
      - cool_kid
      - dork_kid
    incident_report:
      type: Endearing Friendship
      description: Jocks can't hang with nerds!
"""

_EXPECTED_META_PARAMETERS = MetaParameters(
    school_name="Rydell High School",
    cool_kid_to_dork_ratio=0.9,
    hangouts=["corner store", "under the bleachers"],
)
_EXPECTED_PARAMETERS = {
    Namespace(["high_school_movie", "cool_kid"]): {
        Namespace(["is_cool"]): True,
    },
    Namespace(["high_school_movie", "dork_kid"]): {
        Namespace(["catchphrase"]): "This math stuff is easy",
    },
    Namespace(["nearby_theater"]): {
        Namespace(["employees"]): ["cool_kid", "dork_kid"],
        Namespace(["incident_report", "type"]): "Endearing Friendship",
        Namespace(["incident_report", "description"]): "Jocks can't hang with nerds!",
    },
}

_OVERRIDE_PARAMETERS_YAML = """
meta_parameters:
    hangouts: ["detention"]

nearby_theater:
  ros__parameters:
    incident_report:
      description: How wrong I was! Friendship is a beautiful thing.
    lessons_learned: true
"""

_INCOMPLETE_PARAMETERS_YAML = f"""
some_node:
  ros__parameters:
    some_param: {FIELD_PLACEHOLDER}
"""

_NO_META_PARAMETERS_YAML = """
some_node:
    ros__parameters:
        some_param: 3
"""
