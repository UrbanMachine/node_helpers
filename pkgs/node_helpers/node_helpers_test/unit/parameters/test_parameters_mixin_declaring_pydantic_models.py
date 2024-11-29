from enum import Enum
from pathlib import Path
from typing import Any

import pytest
from node_helpers.parameters import ParameterMixin, RequiredParameterNotSetException
from pydantic import BaseModel, Field
from rcl_interfaces.msg import ParameterDescriptor
from rclpy import Parameter


class ParameterNode(ParameterMixin):
    def __init__(self) -> None:
        self.declared: dict[str, tuple[Any, ParameterDescriptor]] = {}
        """Keeps track of declared parameters, for use in assertions."""

        self.config_values: dict[str, Any] = {}
        """If you want declare_parameter to return a particular value for a parameter
        name. This emulating how ROS might return the value of a parameter as defined
        by a parameter file."""

    def declare_parameter(
        self,
        name: str,
        value: Any,
        descriptor: ParameterDescriptor,
        ignore_override: bool,
    ) -> Parameter:
        """Mock the Node.declare_parameter function"""
        assert (
            ignore_override is False
        ), "The ParameterMixin is expected not to ignore parameter overrides"
        assert isinstance(descriptor, ParameterDescriptor)

        # Keep track of declared parameters, to help testing
        self.declared[name] = (self.config_values.get(name, value), descriptor)

        # Return the value as ROS would after having pulled it from configuration
        return Parameter(name=name, value=self.declared[name][0])

    def get_namespace(self) -> str:
        """Used for descriptive logs"""
        return "i'm not really used in these tests yay!"


def test_default_values() -> None:
    """Test that default values work"""

    class Model(BaseModel):
        attr_with_default: str = "ayy"
        attr_without_default: int

    node = ParameterNode()
    with pytest.raises(RequiredParameterNotSetException):
        model = node.declare_from_pydantic_model(
            Model, "cool_prefix", subscribe_to_updates=False
        )

    # Then after setting the value in the "configuration file"
    node.config_values["cool_prefix.attr_without_default"] = 3
    model = node.declare_from_pydantic_model(
        Model, "cool_prefix", subscribe_to_updates=False
    )
    assert model.attr_without_default == 3


def test_field_descriptions() -> None:
    """Test that docstrings are correctly passed into the parameter description"""
    with_description_description_value = "This is a cool value"

    class Model(BaseModel):
        no_description: str = "ayy"
        with_description: int = Field(
            description=with_description_description_value, default=3
        )

    node = ParameterNode()
    node.declare_from_pydantic_model(Model, "prefix", subscribe_to_updates=False)

    assert (
        node.declared["prefix.with_description"][1].description
        == with_description_description_value
    )
    assert node.declared["prefix.no_description"][1].description == ""


def test_prefix_is_added() -> None:
    """Test that empty prefixes are not allowed for pydantic models"""

    expected_value = [1, 2, 3, 4]

    class Model(BaseModel):
        attr_a: list[int]

    node = ParameterNode()
    node.config_values["cool_prefix.attr_a"] = expected_value

    # Try without prefix
    with pytest.raises(ValueError):
        node.declare_from_pydantic_model(Model, "", subscribe_to_updates=False)

    # Try with prefix
    model = node.declare_from_pydantic_model(
        Model, "cool_prefix", subscribe_to_updates=False
    )
    assert node.declared["cool_prefix.attr_a"][0] == expected_value
    assert model.attr_a == expected_value


def test_complex_types() -> None:
    """Test that all ROS types are supported by this system.
    This validates that the parameter mixin isn't just using Pydantic, but also uses
    typing.get_type_hints() behind the scenes.

    Without get_type_hints usage, this test used to fail because pydantic had a more
    complicated way of handling types like List[Type].
    """

    # These are all supported types, as seen in the CONFIG_TO_ROS_MAPPING
    class ManyTypes(BaseModel):
        attr_0: bool = True
        attr_1: int = 3
        attr_2: float = 4.0
        attr_3: str = ""
        attr_4: list[bytes] = [b"1", b"2", b"3"]
        attr_5: list[bool] = [True, False]
        attr_6: list[int] = [1, 2, 3]
        attr_7: list[float] = [1.0, 2.0, 3.0]
        attr_8: list[str] = ["a", "b", "cde"]

    node = ParameterNode()
    assert isinstance(ManyTypes().attr_4, list | tuple)
    assert all(isinstance(v, bytes) for v in ManyTypes().attr_4)

    # The real test is that this call doesn't fail
    node.declare_from_pydantic_model(ManyTypes, "prefix", subscribe_to_updates=False)

    # This is to feel assured that the defaults were pulled and not changed in any way.
    assert node.declared["prefix.attr_0"][0] == ManyTypes().attr_0
    assert node.declared["prefix.attr_1"][0] == ManyTypes().attr_1
    assert node.declared["prefix.attr_2"][0] == ManyTypes().attr_2
    assert node.declared["prefix.attr_3"][0] == ManyTypes().attr_3
    assert node.declared["prefix.attr_4"][0] == ManyTypes().attr_4
    assert node.declared["prefix.attr_5"][0] == ManyTypes().attr_5
    assert node.declared["prefix.attr_6"][0] == ManyTypes().attr_6
    assert node.declared["prefix.attr_7"][0] == ManyTypes().attr_7
    assert node.declared["prefix.attr_8"][0] == ManyTypes().attr_8


def test_nonros_arbitrary_types_are_wrapped() -> None:
    """Test that arbitrary types (e.g. non-ros types) are parsed as ints then wrapped"""

    class ExampleEnum(Enum):
        OPTION_1 = "option_1"
        OPTION_2 = "option_2"

    class ArbitraryTypeModel(BaseModel):
        some_path: Path
        some_enum: ExampleEnum

        # Validates that we can handle Nonetype when it's set as the default value
        some_nonetype: None = None

    node = ParameterNode()
    node.config_values["prefix.some_path"] = "/path/to/something"
    node.config_values["prefix.some_enum"] = "option_2"

    model = node.declare_from_pydantic_model(
        ArbitraryTypeModel, "prefix", subscribe_to_updates=False
    )

    # Validate that the declared values use strings, but the model values are wrapped
    assert node.declared["prefix.some_enum"][0] == "option_2"
    assert node.declared["prefix.some_path"][0] == "/path/to/something"
    assert node.declared["prefix.some_nonetype"][0] is None
    assert model.some_enum is ExampleEnum.OPTION_2
    assert model.some_path == Path("/path/to/something")
    assert model.some_nonetype is None


def test_nested_models() -> None:
    """Tests that we can nest models in other models, and that their parameters are
    prefixed properly
    """

    class ParentModel(BaseModel):
        class ChildModel(BaseModel):
            child_attr: str

        parent_attr: str
        child: ChildModel

    parent_value = "I am a parent"
    child_value = "I'm just a wittle baybee"  # Sorry

    node = ParameterNode()
    node.config_values["prefix.parent_attr"] = parent_value
    node.config_values["prefix.child.child_attr"] = child_value

    node.declare_from_pydantic_model(ParentModel, "prefix", subscribe_to_updates=False)
    assert len(node.declared) == 2
    assert node.declared["prefix.parent_attr"][0] == parent_value
    assert node.declared["prefix.child.child_attr"][0] == child_value


def test_union_types_basic_use() -> None:
    class ExampleEnum(Enum):
        OPTION_1 = "option_1"
        OPTION_2 = "option_2"

    class ModelWithUnions(BaseModel):
        bool_or_int: bool | int
        int_or_str: int | str
        might_be_anything: int | str | list[int] | list[str] | bool
        path_or_int: Path | int
        int_or_enum: int | ExampleEnum
        int_or_enum_with_default: int | ExampleEnum = ExampleEnum.OPTION_1
        list_int_or_none: list[int] | None = None

    node = ParameterNode()
    node.config_values["prefix.might_be_anything"] = [1, 2, 3, 4]
    node.config_values["prefix.bool_or_int"] = True
    node.config_values["prefix.int_or_str"] = "ayy"
    node.config_values["prefix.path_or_int"] = "/path/to/something"
    node.config_values["prefix.int_or_enum"] = "option_2"

    model = node.declare_from_pydantic_model(
        ModelWithUnions, "prefix", subscribe_to_updates=False
    )

    # Validate that the model is correctly declared
    assert model.might_be_anything == [1, 2, 3, 4]
    assert model.bool_or_int is True
    assert model.int_or_str == "ayy"
    assert model.path_or_int == Path("/path/to/something")
    assert model.int_or_enum is ExampleEnum.OPTION_2
    assert model.int_or_enum_with_default is ExampleEnum.OPTION_1
    assert model.list_int_or_none is None
