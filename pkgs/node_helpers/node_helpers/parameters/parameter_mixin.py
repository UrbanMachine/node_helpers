import logging
import typing
from types import GenericAlias, NoneType, UnionType
from typing import Any, TypeVar, cast

from pydantic import BaseModel
from pydantic_core import PydanticUndefined
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterValue
from rclpy.exceptions import ParameterAlreadyDeclaredException, ParameterException
from rclpy.node import Node
from rclpy.parameter import Parameter

from .parsing import ParsableType, get_parser

PydanticType = TypeVar("PydanticType", bound=BaseModel)


class RequiredParameterNotSetException(Exception):
    """Raised when a parmeter was marked as required, but was not set. This can occur
    if the user hasn't set the parameter via the launch file or command line."""


class ParameterMixin:
    """This mixin enables declaring parameters within a node in a more sane manner
    than what rclpy allows by default.

    The benefits of using this mixin is:
    1) Less boilerplate
    2) Support for parsing parameters into Pydantic models!
    3) Type checking!
    4) Validate that the parameter is _actually_ set from outside source, when
       `required=True` is set.

    """

    def declare_and_get_parameter(
        self: Node,
        name: str,
        type_: type[ParsableType],
        description: str = "",
        default_value: ParsableType | None = None,
        required: bool = False,
    ) -> ParsableType:
        """
        Declare a parameter and immediately receive the (current) value of the param,
        where the main use case is to declare a parameter that will be filled in the
        launch file.

        :param name: The name of the parameter.
        :param type_: What type should be returned by this function. Must have a
            supported parser!
        :param description: The ROS parameter description.
        :param default_value: The default to set this parameter to. Must be None if
            required=True.
        :param required: If True, then this function will raise an error if the value
            was not assigned from an outside source. Use this for values that don't make
            sense to have defaults for, but that are also necessary for normal usage.

        :raises RequiredParameterNotSetException: If the parameter is marked as
            `required` but is not set by the launch file, this will be raised.
        :raises TypeError: If a given value doesn't match the specified type
        :raises ValueError: If the parameters to the function were invalid
        :raises UnfilledParametersFileError: If an unfilled sentinel value is present
            in one of the parameters

        :return: The current value of the parameter
        """

        parser = get_parser(type_)

        if default_value is not None:
            # Convert default_value to the 'config' type
            default_value_config = parser.as_config_type(default_value)

            if required:
                raise ValueError(
                    "If required=True, then default_value must be None. Got "
                    f"{default_value=}"
                )

            # Validate the default_value matches the specified type
            if not required and not parser.ros_type.check(default_value_config):
                raise TypeError(
                    f"The default value {default_value} for parameter '{name}' does not"
                    f" match the specified type {parser.ros_type}"
                )
        else:
            default_value_config = None

        # Declare the parameter and retrieve the current value
        descriptor = ParameterDescriptor(
            description=description, type=parser.ros_type.value, dynamic_typing=True
        )

        retrieved_value = self.declare_parameter(
            name=name,
            value=default_value_config,
            descriptor=descriptor,
            ignore_override=False,
        ).value

        if required and retrieved_value is None:
            raise RequiredParameterNotSetException(
                f"The parameter '{name}' is required to be set externally. Please "
                f"specify this parameter in the launch or configuration file. "
                f"Namespace: {self.get_namespace()}"
            )

        if (
            isinstance(retrieved_value, str)
            and FIELD_PLACEHOLDER in retrieved_value.lower()
        ):
            raise UnfilledParametersFileError(
                f"Field '{name!s}' has not been filled in! Please fill in any fields"
                f" with the value '{FIELD_PLACEHOLDER}' with an actual value."
            )

        if not parser.ros_type.check(retrieved_value):
            raise TypeError(
                f"The parameter '{name}' was set with an incorrect type. The specified "
                f"type was {parser.ros_type}, but the final value had a type of "
                f"{type(retrieved_value)} and value of '{retrieved_value!s}'"
            )

        return parser.from_config_type(retrieved_value)

    def subscribe_attribute_to_updates(
        self: Node,
        attr: str,
        parameter_name: str,
        type_: type[ParsableType],
        object_with_attr: object | None = None,
    ) -> None:
        """Subscribe an attribute on an object to updates for when a parameter gets
        changed.

        :param attr: The attribute to update on the object
        :param parameter_name: The parameter name to use for updates
        :param type_: What type will the updated value be parsed as
        :param object_with_attr: The object that has the attribute. If none is set,
            it is assumed to be `self`.
        :raises AttributeError: If the `object_with_attr` does not in fact have the
            specified `attr`.
        """

        object_with_attr = object_with_attr or self
        pretty_attribute_name = f"{object_with_attr.__class__.__name__}.{attr}"

        if not hasattr(object_with_attr, attr):
            raise AttributeError(f"The attribute {pretty_attribute_name} was not found")

        def on_set_parameter(parameters: list[Parameter]) -> SetParametersResult:
            parser = get_parser(type_)
            for parameter in parameters:
                if parameter.name != parameter_name:
                    continue

                logging.info(
                    f"Setting {pretty_attribute_name}={parameter.value}. Relevant "
                    f"parameter name='{parameter_name}'"
                )

                parsed_value = parser.from_config_type(parameter.value)
                setattr(object_with_attr, attr, parsed_value)
                break

            return SetParametersResult(successful=True)

        self.add_on_set_parameters_callback(callback=on_set_parameter)

    def declare_from_pydantic_model(
        self, model: type[PydanticType], prefix: str, subscribe_to_updates: bool = True
    ) -> PydanticType:
        """
        :param model: The pydantic model to instantiate from
        :param prefix: The parameter name prefix. For example, if model.cool_attribute
            exists and the prefix is "stepper_1", then in configuration cool_attribute
            would be set as

            stepper_1:
                cool_attribute: value
        :param subscribe_to_updates: When True, all attributes of the pydantic model
            will be capable of being dynamically updates using Node.set_parameters()
        :raises ValueError: If the prefix is an empty string
        :return: An instantiated model
        """
        if prefix == "":
            raise ValueError("Empty prefixes are not allowed for pydantic parameters!")

        model_kwargs = {}
        subscribable_attributes: list[tuple[str, type, str]] = []

        for attr_name, field in model.model_fields.items():
            type_ = cast(type[Any], field.annotation)
            parameter_name = f"{prefix}.{attr_name}"

            # Generic aliases like list[int] don't work with issubclass, so we need
            # this special case
            is_union = isinstance(type_, UnionType)
            try_types = [type_] if not is_union else typing.get_args(type_)
            is_pydantic_model, parsed_value, parsed_type = self._try_to_parse_types(
                parameter_name=parameter_name,
                possible_types=try_types,
                default_value=field.get_default(),
                required=field.is_required(),
                description=field.description or "",
                subscribe_to_updates=subscribe_to_updates,
            )
            model_kwargs[attr_name] = parsed_value

            if not is_pydantic_model:
                subscribable_attributes.append((attr_name, parsed_type, parameter_name))

        instantiated: PydanticType = model(**model_kwargs)

        # Subscribe the instantiated object to parameter updates
        if subscribe_to_updates:
            for attr_name, parseable_type, parameter_name in subscribable_attributes:
                self.subscribe_attribute_to_updates(
                    attr=attr_name,
                    parameter_name=parameter_name,
                    type_=parseable_type,
                    object_with_attr=instantiated,
                )
        return instantiated

    def _try_to_parse_types(
        self,
        parameter_name: str,
        possible_types: typing.Sequence[type],
        default_value: Any,
        required: bool,
        subscribe_to_updates: bool,
        description: str = "",
    ) -> tuple[bool, Any, type]:
        """Parse the type and return (is_pydantic_model, parsed_value, parsed_type)"""

        last_exception: BaseException | None = None

        if None in possible_types and possible_types[-1] is not None:
            raise ValueError(
                "None must be the last element in any pydantic model with Union types"
            )

        # 'declare_and_get_parameter' cannot use PydanticUndefined as a default value,
        # so we use None and check at the end if the default value was 'None'
        default = None if default_value is PydanticUndefined else default_value
        for try_type in possible_types:
            is_list = isinstance(try_type, GenericAlias)
            is_basemodel = False if is_list else issubclass(try_type, BaseModel)

            try:
                if is_basemodel:
                    parsed_value: Any = self.declare_from_pydantic_model(
                        model=try_type,
                        prefix=parameter_name,
                        subscribe_to_updates=subscribe_to_updates,
                    )
                    return True, parsed_value, try_type
                else:
                    parsed_value = self.declare_and_get_parameter(
                        name=parameter_name,
                        type_=try_type,
                        description=description,
                        default_value=default,
                        required=required,
                    )
                    return False, parsed_value, try_type

            except ParameterAlreadyDeclaredException:
                # This is a special case where a pydantic model has None set as the
                # default value. In this case, we make sure that None was 'try_type',
                # and just return None as the parsed value
                if default_value is None and try_type is NoneType:
                    logging.debug(
                        f"Parameter '{parameter_name}' was already declared, but the "
                        f"default value was None. Using None as the parsed value."
                    )
                    return False, None, try_type
                raise
            except (
                ParameterException,
                TypeError,
                RequiredParameterNotSetException,
            ) as e:
                # This exception should never happen for non-pydantic models
                if not is_basemodel and e is RequiredParameterNotSetException:
                    raise

                # This is a common case where the parameter didn't match the type.
                # We will continue trying to parse the parameter with the next type
                logging.debug(
                    f"Couldn't parse {parameter_name} using type {try_type}, out of "
                    f"possible types {possible_types}. Reason {type(e)}('{e}')"
                )
                last_exception = e

        assert last_exception is not None
        raise last_exception


FIELD_PLACEHOLDER = "<override this>"
"""The placeholder text put on parameters that need to be filled in manually"""


class UnfilledParametersFileError(Exception):
    """One or more fields were left unfilled in the resulting parameters file"""
