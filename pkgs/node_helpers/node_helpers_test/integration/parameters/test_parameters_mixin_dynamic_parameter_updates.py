from collections.abc import Generator
from enum import Enum
from pathlib import Path
from typing import Any

import pytest
from node_helpers.parameters import ParameterMixin
from node_helpers.testing import set_up_node
from pydantic import BaseModel
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

PARAMETER_NAME_A = "parameter_a"
PARAMETER_VALUE_A = 1.4
PARAMETER_NAME_B = "parameter_b"
PARAMETER_VALUE_B = "test"


class ParameterNode(Node, ParameterMixin):
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__("parameters_node", *args, **kwargs)
        self.a_parameter_attr = self.declare_and_get_parameter(
            name=PARAMETER_NAME_A, type_=float, required=True
        )
        self.b_parameter_attr = self.declare_and_get_parameter(
            name=PARAMETER_NAME_B, type_=str, required=True
        )


@pytest.fixture()
def parameter_node() -> Generator[ParameterNode, None, None]:
    yield from set_up_node(
        node_class=ParameterNode,
        namespace="cool_params",
        node_name="parameters_node",
        parameter_overrides=[
            Parameter(name=PARAMETER_NAME_A, value=PARAMETER_VALUE_A),
            Parameter(name=PARAMETER_NAME_B, value=PARAMETER_VALUE_B),
        ],
    )


def test_successful_subscribe_attribute_to_updates(
    parameter_node: ParameterNode,
) -> None:
    assert parameter_node.a_parameter_attr == PARAMETER_VALUE_A

    # Dynamically update the parameter, and assert nothing changes (yet)
    parameter_node.set_parameters([Parameter(name=PARAMETER_NAME_A, value=432.619)])
    assert parameter_node.a_parameter_attr == PARAMETER_VALUE_A

    # Subscribe the attribute to updates
    parameter_node.subscribe_attribute_to_updates(
        "a_parameter_attr", PARAMETER_NAME_A, float
    )

    # Previous changes shouldn't have affected the parameter
    assert parameter_node.a_parameter_attr == PARAMETER_VALUE_A

    # Publish a new parameter change, now it should have changed
    results: list[SetParametersResult] = parameter_node.set_parameters(
        [Parameter(name=PARAMETER_NAME_A, value=1234.5678)]
    )
    assert len(results) == 1
    assert results[0].successful
    assert parameter_node.a_parameter_attr == 1234.5678


def test_setting_unsubscribed_attribute_returns_failure_message(
    parameter_node: ParameterNode,
) -> None:
    """Test that if one attribute is subscribed and another one isn't, that if the one
    that isn't is attempted to be set, that a failure message will be received.

    Then subscribe both, and try again.
    """
    # Subscribe attribute A but don't subscribe attribute B
    parameter_node.subscribe_attribute_to_updates(
        "a_parameter_attr", PARAMETER_NAME_A, float
    )

    # Try updating both A and B
    results = parameter_node.set_parameters(
        [
            Parameter(name=PARAMETER_NAME_A, value=5.0),
            Parameter(name=PARAMETER_NAME_B, value="cool-string"),
        ],
    )
    assert len(results) == 2

    # Check that what should/shouldn't have changed happened
    assert parameter_node.a_parameter_attr == 5.0
    assert parameter_node.b_parameter_attr == PARAMETER_VALUE_B

    # Now subscribe B and try again
    parameter_node.subscribe_attribute_to_updates(
        "b_parameter_attr", PARAMETER_NAME_B, str
    )

    # Try updating both A and B
    results = parameter_node.set_parameters(
        [
            Parameter(name=PARAMETER_NAME_A, value=10.0),
            Parameter(name=PARAMETER_NAME_B, value="cool-string-2"),
        ],
    )
    assert len(results) == 2

    # Check that what should/shouldn't have changed happened
    assert parameter_node.a_parameter_attr == 10.0
    assert parameter_node.b_parameter_attr == "cool-string-2"


def test_setting_same_parameter_twice(parameter_node: ParameterNode) -> None:
    """Test that when setting the same parameter twice, the last value is used."""
    parameter_node.subscribe_attribute_to_updates(
        "a_parameter_attr", PARAMETER_NAME_A, float
    )
    parameter_node.set_parameters(
        [
            Parameter(name=PARAMETER_NAME_A, value=100.1),
            Parameter(name=PARAMETER_NAME_A, value=100.2),
            Parameter(name=PARAMETER_NAME_A, value=100.3),
            Parameter(name=PARAMETER_NAME_A, value=100.4),
        ]
    )
    assert parameter_node.a_parameter_attr == 100.4


def test_invalid_attribute_raises_error(parameter_node: ParameterNode) -> None:
    # Success case
    parameter_node.subscribe_attribute_to_updates(
        "a_parameter_attr", PARAMETER_NAME_A, float
    )

    # Failure case
    with pytest.raises(AttributeError):
        parameter_node.subscribe_attribute_to_updates(
            "nonexistent_attr", PARAMETER_NAME_A, float
        )


def test_pydantic_parameter_updating(parameter_node: ParameterNode) -> None:
    """Test that parameter updates are received by ros and then updated directly on
    the pydantic object.
    """

    class CoolModel(BaseModel):
        param_a: str = "default_value_a"
        param_b: str = "default_value_b"

    model = parameter_node.declare_from_pydantic_model(CoolModel, "cool_prefix")
    assert model.param_a == "default_value_a"
    assert model.param_b == "default_value_b"

    # Try updating one value
    parameter_node.set_parameters([Parameter(name="cool_prefix.param_a", value="try1")])
    assert model.param_a == "try1"
    assert model.param_b == "default_value_b"

    # Try updating the other value
    parameter_node.set_parameters([Parameter(name="cool_prefix.param_b", value="try2")])
    assert model.param_a == "try1"
    assert model.param_b == "try2"

    # Try updating both values at once
    parameter_node.set_parameters(
        [
            Parameter(name="cool_prefix.param_a", value="try3a"),
            Parameter(name="cool_prefix.param_b", value="try3b"),
        ]
    )
    assert model.param_a == "try3a"
    assert model.param_b == "try3b"


def test_nonros_arbitrary_type_parameters_updating(
    parameter_node: ParameterNode,
) -> None:
    """Test that arbitrary types like Path or Enum are allowed, and when updated they
    maintain the desired arbitrary type."""

    class ExampleEnum(Enum):
        OPTION_1 = "option_1"
        OPTION_2 = "option_2"

    original_path_value = Path("/a/path/somewhere")
    original_enum_value = ExampleEnum.OPTION_2

    class ArbitraryTypeModel(BaseModel):
        some_path: Path = original_path_value
        some_enum: ExampleEnum = original_enum_value

    model = parameter_node.declare_from_pydantic_model(
        ArbitraryTypeModel, "cool_prefix"
    )
    assert model.some_path == original_path_value
    assert model.some_enum == original_enum_value

    # Set the path parameter using a string, and validate it was re-converted to a Path
    updated_path_raw_value = "/a/new/path/somewhere"
    parameter_node.set_parameters(
        [Parameter(name="cool_prefix.some_path", value=updated_path_raw_value)]
    )
    assert isinstance(model.some_path, Path)
    assert model.some_path == Path(updated_path_raw_value)

    # Set the enum parameter using a string, and validate it was re-converted to a Enum
    updated_enum_raw_value = "option_1"
    parameter_node.set_parameters(
        [Parameter(name="cool_prefix.some_enum", value=updated_enum_raw_value)]
    )
    assert isinstance(model.some_enum, ExampleEnum)
    assert model.some_enum == ExampleEnum(updated_enum_raw_value)


def test_subscribe_to_updates_argument_is_respected(
    parameter_node: ParameterNode,
) -> None:
    """Test that the `subscribe_to_updates` in declare_from_pydantic_model is
    respected."""

    class ChildModel(BaseModel):
        third_value: str = "ayy"
        fourth_value: str = "bingobongo"

    class SomeModel(BaseModel):
        some_value: str = "value"
        another_value: int = 3

        # Test that the subscribe_to_updates argument is recursively passed to children
        child: ChildModel

    assert len(parameter_node._on_set_parameters_callbacks) == 1

    # Test disabling updates
    parameter_node.declare_from_pydantic_model(
        SomeModel, "some_prefix", subscribe_to_updates=False
    )
    assert len(parameter_node._on_set_parameters_callbacks) == 1

    # Test enabling updates
    parameter_node.declare_from_pydantic_model(
        SomeModel, "another_prefix", subscribe_to_updates=True
    )
    assert len(parameter_node._on_set_parameters_callbacks) == 5
