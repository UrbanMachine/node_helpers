from unittest import mock

import pytest
from node_helpers.parameters import (
    FIELD_PLACEHOLDER,
    ParameterMixin,
    RequiredParameterNotSetException,
    UnfilledParametersFileError,
)


def test_required_disallows_default_value() -> None:
    """Test that setting a default value is not allowed when required=True"""
    with pytest.raises(ValueError):
        ParameterMixin().declare_and_get_parameter(
            name="test",
            required=True,
            type_=str,
            default_value="Uh oh, this shouldn't be allowed!",
        )


def test_default_value_doesnt_match_assigned_type() -> None:
    """Test that if the default_value doesn't match the documented parameter_type,
    a TypeError is raised."""
    with pytest.raises(TypeError):
        ParameterMixin().declare_and_get_parameter(
            name="coolparam", default_value=3.0, type_=str
        )


def test_required_raises_error_if_not_set() -> None:
    """Test that if required=True and the value is not set, that an error is
    raised.
    """

    class CoolNode(mock.MagicMock, ParameterMixin):
        pass

    # Mock a node that will be pre-filled with an incorrect type
    node = CoolNode()
    parameter_mock = mock.MagicMock()
    parameter_mock.value = None
    node.declare_parameter.return_value = parameter_mock

    with pytest.raises(RequiredParameterNotSetException):
        node.declare_and_get_parameter(
            name="test",
            required=True,
            type_=float,
        )


def test_incorrect_type() -> None:
    """Test that if a user described the parameter as being a certain type, but then
    configures it with another type, that an error is raised."""

    class CoolNode(mock.MagicMock, ParameterMixin):
        pass

    # Mock a node that will be pre-filled with an incorrect type
    node = CoolNode()
    parameter_mock = mock.MagicMock()
    parameter_mock.value = "This is a string type!!!"
    node.declare_parameter.return_value = parameter_mock

    with pytest.raises(TypeError):
        node.declare_and_get_parameter(
            name="test",
            required=True,
            type_=float,
        )


def test_unfilled_detection() -> None:
    """Test that fields with a sentinel placeholder value cause an error"""

    class CoolNode(mock.MagicMock, ParameterMixin):
        pass

    # Mock a node that will be pre-filled with a placeholder value
    node = CoolNode()
    parameter_mock = mock.MagicMock()
    parameter_mock.value = FIELD_PLACEHOLDER
    node.declare_parameter.return_value = parameter_mock

    with pytest.raises(UnfilledParametersFileError):
        node.declare_and_get_parameter(
            name="test",
            required=True,
            type_=str,
        )
