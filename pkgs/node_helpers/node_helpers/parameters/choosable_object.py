"""
In short, this module provides a seamless way to select Classes and Instances via
configuration, by having a global registration system and an easy way to register and
retrieve them.

Take a look at the node_helpers/parameters documentation for full usage instructions.
"""

from collections import defaultdict
from typing import Any, TypeVar, cast

from pydantic import GetCoreSchemaHandler
from pydantic_core import CoreSchema, core_schema

SomeBaseClassType = TypeVar("SomeBaseClassType", bound="Choosable")

# Define the global registry of uninstantiated classes
ClassRegistryType = defaultdict[type["Choosable"], dict[str, type["Choosable"]]]
_global_choosable_class_registry: ClassRegistryType = defaultdict(dict)
"""Global registry of all classes that can be loaded from configuration.

This is a dictionary of dictionaries. The first key is the base class of the class
being loaded. The second key is the name of the class being loaded. The value is the
class itself.
"""

# Define the global registry of instantiated instances
InstanceRegistryType = defaultdict[type["Choosable"], dict[str, "Choosable"]]
_global_choosable_instance_registry: InstanceRegistryType = defaultdict(dict)
"""Global registry of all instances that can be loaded from configuration."""


class UnregisteredChoosableError(Exception):
    """Used when a class or instance is not registered in the global registry"""


class DuplicateRegistrationError(Exception):
    """Used when a class or instance is registered but the name has already been used"""


class Choosable:
    """Base Class for any class that wants to be dynamically choosable by a user in
    configuration."""

    def __init_subclass__(
        cls, registered_name: str | None = None, **kwargs: Any
    ) -> None:
        """
        :param registered_name: The name of the class that was registered. This is
            the name that should be used in configuration.
            By default this is the classes name, but subclasses can override
            this.
        :param kwargs: Other metaclass parameters, if any
        :raises DuplicateRegistrationError: if the class was already registered
        """
        super().__init_subclass__(**kwargs)
        registered_name = registered_name or cls.__name__
        base_choosable_class = cls._find_base_choosable_class()
        registry_for_class = _global_choosable_class_registry[base_choosable_class]

        if registered_name in registry_for_class:
            raise DuplicateRegistrationError(
                f"A class with the name {registered_name} has already been registered "
                f"for {base_choosable_class.__name__}. Make sure you're not declaring "
                f"multiple classes with the same name and same base class."
            )
        registry_for_class[registered_name] = cls

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: GetCoreSchemaHandler
    ) -> CoreSchema:
        """This allows pydantic to accept instances of Choosable as
        valid configuration values. Otherwise, you would have to specify the
        'allow_arbitrary_types' for any pydantic model that declares an instance of a
        Choosable.

        Read more here:
        https://docs.pydantic.dev/latest/concepts/types/#custom-types
        # noqa: DAR101
        # noqa: DAR201
        """
        return core_schema.is_instance_schema(Choosable)

    @classmethod
    def get_registered_child_class(
        cls: type[SomeBaseClassType], name: str
    ) -> "type[SomeBaseClassType]":
        """Get a registered class by name that subclasses this class.

        Usage:
        >>> class BaseChoosableClass(Choosable):
        >>>    pass
        >>>
        >>> class MyChoosableClass(BaseChoosableClass):
        >>>     pass
        >>>
        >>> cls_ = BaseChoosableClass.get_registered_child_class("MyChoosableClass")
        >>> assert cls_ == MyChoosableClass

        :param name: The name of the class to retrieve
        :return: The class that was registered with the given name
        :raises UnregisteredChoosableError: If the class was not registered
        """
        base_choosable_class = cls._find_base_choosable_class()
        registry_for_class = _global_choosable_class_registry[base_choosable_class]

        try:
            return cast(type[SomeBaseClassType], registry_for_class[name])
        except KeyError as e:
            raise UnregisteredChoosableError(
                f"{cls.__name__} with name '{name}' was not registered. Make sure "
                f"{cls.__name__} has a parent that inherits from "
                f"Choosable so that it's added to the global registry. "
                f"Registered classes for '{base_choosable_class.__name__}' are: "
                f"{list(registry_for_class.keys())}"
            ) from e

    @classmethod
    def _find_base_choosable_class(
        cls: type[SomeBaseClassType],
    ) -> type[SomeBaseClassType]:
        """Find the base class that subclasses Choosable. This will be
        used as a key for the global registry.

        :return: The class that subclasses Choosable
        :raises RuntimeError: If the class doesn't subclass Choosable
        """
        # Exit if this cls is already the base inheritor
        is_root_class = Choosable in cls.__bases__
        if is_root_class:
            return cls

        for base in cls.__bases__:
            # Recursively check the base classes of the current base class
            if issubclass(base, Choosable):
                recursive_base = base._find_base_choosable_class()  # noqa: SLF001
                if recursive_base:
                    return cast(type[SomeBaseClassType], recursive_base)

        raise RuntimeError(
            "This shouldn't happen. There should always exist a base class subclassing "
            "Choosable. Are you calling this method directly on "
            "Choosable? If so, you should inherit it and then call it on"
            " a subclass."
        )

    def register_instance(self, name: str) -> None:
        """Register this instance with the global registry, with the given name
        :param name: The name to register the instance under
        :raises DuplicateRegistrationError: if the instance was already registered
        """
        base_choosable_class = self._find_base_choosable_class()
        registry_for_class = _global_choosable_instance_registry.setdefault(
            base_choosable_class, {}
        )

        if name in registry_for_class:
            raise DuplicateRegistrationError(
                f"An instance with the name {name} has already been registered for "
                f"{base_choosable_class.__name__}. Make sure you're not declaring "
                f"multiple instances with the same name and same base class."
            )
        registry_for_class[name] = self

    @classmethod
    def get_registered_instance(
        cls: type[SomeBaseClassType], name: str
    ) -> SomeBaseClassType:
        """Get a registered instance of this class"""
        base_choosable_class = cls._find_base_choosable_class()
        registry_for_class = _global_choosable_instance_registry[base_choosable_class]

        try:
            return cast(SomeBaseClassType, registry_for_class[name])
        except KeyError as ex:
            raise UnregisteredChoosableError(
                f"{cls.__name__} with name '{name}' was not registered. Make sure "
                f"{cls.__name__} is instantiated with the register_instance method "
                f"so that it's added to the global registry. "
                f"Registered instances for '{base_choosable_class.__name__}' are: "
                f"{list(registry_for_class.keys())}"
            ) from ex

    @classmethod
    def get_registered_class_name(cls: type[SomeBaseClassType]) -> str:
        """Get the name of the registered class"""
        base_choosable_class = cls._find_base_choosable_class()
        registry_for_class = _global_choosable_class_registry[base_choosable_class]
        return next(name for name, type_ in registry_for_class.items() if type_ is cls)

    def get_registered_instance_name(self) -> str:
        """Get the name of the registered instance"""
        base_choosable_class = self._find_base_choosable_class()
        registry_for_instance = _global_choosable_instance_registry[
            base_choosable_class
        ]
        try:
            return next(
                name
                for name, instance in registry_for_instance.items()
                if instance is self
            )
        except StopIteration as ex:
            raise UnregisteredChoosableError(
                f"The instance of {self.__class__.__name__} was not registered. Make "
                f"sure {self.__class__.__name__} is registered by calling the "
                f"register_instance method so that it's added to the global registry. "
                f"Registered instances for '{base_choosable_class.__name__}' are: "
                f"{list(registry_for_instance.keys())}"
            ) from ex
