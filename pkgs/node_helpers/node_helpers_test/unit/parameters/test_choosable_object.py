from abc import ABC, abstractmethod
from collections.abc import Generator

import pytest
from node_helpers.parameters import Choosable, choosable_object
from node_helpers.parameters.choosable_object import (
    ClassRegistryType,
    DuplicateRegistrationError,
    InstanceRegistryType,
    UnregisteredChoosableError,
)
from pydantic import BaseModel

from .test_parameters_mixin_declaring_pydantic_models import ParameterNode


@pytest.fixture()
def clean_registries() -> (
    Generator[tuple[ClassRegistryType, InstanceRegistryType], None, None]
):
    """Return an unmodified global class and instance registry before each test"""
    # Copy the contents of the registries before the test
    class_registry = choosable_object._global_choosable_class_registry
    instance_registry = choosable_object._global_choosable_instance_registry
    classes_before = class_registry.copy()
    instances_before = instance_registry.copy()

    # Clear the registries before and after each test
    class_registry.clear()
    instance_registry.clear()
    yield class_registry, instance_registry
    class_registry.clear()
    instance_registry.clear()

    # Re add items to the class and instance registries
    class_registry.update(classes_before)
    instance_registry.update(instances_before)


def test_pydantic_parsing_integration_for_classes(
    clean_registries: tuple[ClassRegistryType, InstanceRegistryType],
) -> None:
    """Test that classes can be chosen via configuration, using parameter: type[Class]
    in a pydantic model.
    """

    class BaseSomeClassThatIsChoosable(Choosable, ABC):
        """Test that the base class pattern still works with choosable classes"""

        @abstractmethod
        def some_method(self) -> str:
            pass

    assert len(choosable_object._global_choosable_class_registry) == 1

    class SomeChoosableClassImplementation(BaseSomeClassThatIsChoosable):
        def some_method(self) -> str:
            return "hey this shouldn't be called"

    assert len(choosable_object._global_choosable_class_registry) == 1

    class AnotherChoosableClassImplementation(BaseSomeClassThatIsChoosable):
        def some_method(self) -> str:
            return "correct!"

    # Validate the registry now has these classes
    assert choosable_object._global_choosable_class_registry == {
        BaseSomeClassThatIsChoosable: {
            "BaseSomeClassThatIsChoosable": BaseSomeClassThatIsChoosable,
            "SomeChoosableClassImplementation": SomeChoosableClassImplementation,
            "AnotherChoosableClassImplementation": AnotherChoosableClassImplementation,
        }
    }

    class ParametersWithChoosableClass(BaseModel):
        choosable_class_name: type[BaseSomeClassThatIsChoosable]

    node = ParameterNode()
    node.config_values["a.choosable_class_name"] = "AnotherChoosableClassImplementation"

    model = node.declare_from_pydantic_model(
        ParametersWithChoosableClass, "a", subscribe_to_updates=False
    )

    # Validate that the declared values use strings, but the class was correctly chosen
    assert (
        node.declared["a.choosable_class_name"][0]
        == "AnotherChoosableClassImplementation"
    )
    assert model.choosable_class_name is AnotherChoosableClassImplementation
    assert model.choosable_class_name().some_method() == "correct!"


def test_pydantic_parsing_integration_for_instances(
    clean_registries: tuple[ClassRegistryType, InstanceRegistryType],
) -> None:
    """Test that (registered) instances of a class can be chosen via configuration,
    using parameter: Class in a pydantic model.
    """

    class BaseChoosableInstance(Choosable):
        pass

    class InheritorChoosableInstance(BaseChoosableInstance):
        pass

    class ParametersWithChoosableInstance(BaseModel):
        choosable_instance: BaseChoosableInstance  # note it's not wrapped in a type[]

    # Register a few instances
    instance_1 = BaseChoosableInstance()
    instance_2 = InheritorChoosableInstance()
    instance_3 = BaseChoosableInstance()
    instance_1.register_instance("instance_1")
    instance_2.register_instance("instance_2")
    instance_3.register_instance("instance_3")

    node = ParameterNode()
    node.config_values["a.choosable_instance"] = "instance_2"

    model = node.declare_from_pydantic_model(
        ParametersWithChoosableInstance, "a", subscribe_to_updates=False
    )

    # Validate the chosen instance is was retrieved from the global registry based on
    # the users configuration values
    assert model.choosable_instance is instance_2
    assert node.declared["a.choosable_instance"][0] == "instance_2"

    # Now try declaring an instance that doesn't exist
    node.config_values["a.choosable_instance"] = "instance_4"
    with pytest.raises(UnregisteredChoosableError):
        model = node.declare_from_pydantic_model(ParametersWithChoosableInstance, "a")


def test_descriptive_error_is_returned_when_class_not_registered(
    clean_registries: tuple[ClassRegistryType, InstanceRegistryType],
) -> None:
    class BaseChoosable(Choosable):
        pass

    class RegisteredClassA(BaseChoosable):
        pass

    class RegisteredClassB(BaseChoosable):
        pass

    assert (
        BaseChoosable.get_registered_child_class("RegisteredClassA") is RegisteredClassA
    )
    assert (
        BaseChoosable.get_registered_child_class("RegisteredClassB") is RegisteredClassB
    )

    # This class isn't defined anywhere, so it should raise an error
    with pytest.raises(UnregisteredChoosableError):
        BaseChoosable.get_registered_child_class("RegisteredClassC")

    # This class is defined but only under the child class, so it should raise an error
    with pytest.raises(RuntimeError):
        Choosable.get_registered_child_class("RegisteredClassA")
    with pytest.raises(RuntimeError):
        Choosable.get_registered_child_class("RegisteredClassB")


def test_registration_happens_with_basest_choosable_class(
    clean_registries: tuple[ClassRegistryType, InstanceRegistryType],
) -> None:
    """Test that if there's a chain of subclassing, the 'root' inheritor of
    Choosable is the one that gets registered
    """

    class BaseChoosable(Choosable):
        pass

    class RegisteredClassA(BaseChoosable):
        pass

    class RegisteredClassB(RegisteredClassA):
        pass

    assert (
        BaseChoosable.get_registered_child_class("RegisteredClassA") is RegisteredClassA
    )

    # Try accessing RegisteredClassB from the base class and the A class (both work)
    assert (
        BaseChoosable.get_registered_child_class("RegisteredClassB") is RegisteredClassB
    )
    assert (
        RegisteredClassB.get_registered_child_class("RegisteredClassB")
        is RegisteredClassB
    )
    assert (
        RegisteredClassA.get_registered_child_class("RegisteredClassB")
        is RegisteredClassB
    )


def test_custom_registered_name(
    clean_registries: tuple[ClassRegistryType, InstanceRegistryType],
) -> None:
    """Test that if there's a chain of subclassing, the 'root' inheritor of
    Choosable is the one that gets registered
    """

    class BaseChoosable(Choosable):
        pass

    class CustomNameClass(BaseChoosable, registered_name="custom_name"):
        pass

    class DefaultNameClass(BaseChoosable):
        pass

    assert BaseChoosable.get_registered_child_class("custom_name") is CustomNameClass
    assert (
        BaseChoosable.get_registered_child_class("DefaultNameClass") is DefaultNameClass
    )


def test_instance_registration_is_scoped(
    clean_registries: tuple[ClassRegistryType, InstanceRegistryType],
) -> None:
    """Test that instances can be registered and retrieved, using scoping rules"""

    class BaseA(Choosable):
        pass

    class ImplA(BaseA):
        pass

    class BaseB(Choosable):
        pass

    class ImplB(BaseB):
        pass

    # Shotgun approach to registering instances. The real check here is to ensure that
    # the instances are registered in the correct scope, and that both base and child
    # classes can reach the 'scoped registry' for their base class
    instance_a_1 = ImplA()
    instance_a_2 = ImplA()  # noqa: F841
    instance_a_3 = BaseA()
    instance_b_1 = ImplB()
    instance_b_2 = ImplB()  # noqa: F841
    instance_b_3 = BaseB()

    class_registry, instance_registry = clean_registries
    assert len(class_registry) == 2
    assert len(instance_registry) == 0

    # Add two instances of A, and validate that they live under the same scope
    instance_a_1.register_instance("test_a_1")
    assert len(instance_registry) == 1
    instance_a_3.register_instance("test_a_3")
    assert len(instance_registry) == 1
    assert len(class_registry) == 2
    assert BaseA.get_registered_instance("test_a_1") is instance_a_1
    assert BaseA.get_registered_instance("test_a_3") is instance_a_3
    assert instance_a_3.get_registered_instance("test_a_1") is instance_a_1
    assert instance_a_3.get_registered_instance("test_a_3") is instance_a_3

    # Now validate that if a B instance is created, it lives under a new scope
    instance_b_1.register_instance("test_b_1")
    instance_b_3.register_instance("test_b_3")
    assert len(instance_registry) == 2
    assert BaseB.get_registered_instance("test_b_1") is instance_b_1
    assert ImplB.get_registered_instance("test_b_3") is instance_b_3

    # Now validate that failures occur if you try to access the wrong scope
    with pytest.raises(UnregisteredChoosableError):
        BaseA.get_registered_instance("test_b_1")

    with pytest.raises(UnregisteredChoosableError):
        ImplB.get_registered_instance("test_a_1")


def test_reusing_the_same_name_fails_for_classes(
    clean_registries: tuple[ClassRegistryType, InstanceRegistryType],
) -> None:
    """Test you aren't allowed to register two classes with the same name"""

    class BaseA(Choosable):
        pass

    # This is okay, because two root 'scopes' with the same name can exist
    class BaseA(Choosable):  # type: ignore  # noqa: F811
        pass

    class ImplA(BaseA):
        pass

    # This however is not okay, because two classes with the same name and shared scope
    # should not exist
    with pytest.raises(DuplicateRegistrationError):

        class ImplA(BaseA):  # type: ignore # noqa: F811
            pass

    # This is fine however, because a custom name is used
    class ImplA(BaseA, registered_name="custom_name"):  # type: ignore # noqa: F811
        pass


def test_reusing_same_name_fails_for_instances(
    clean_registries: tuple[ClassRegistryType, InstanceRegistryType],
) -> None:
    """Test you aren't allowed to register two instances with the same name"""

    class Base(Choosable):
        pass

    class SomeInstance(Base):
        pass

    instance_1 = SomeInstance()
    instance_2 = SomeInstance()
    instance_3 = Base()

    instance_1.register_instance("instance_1")

    with pytest.raises(DuplicateRegistrationError):
        instance_1.register_instance("instance_1")
    with pytest.raises(DuplicateRegistrationError):
        instance_2.register_instance("instance_1")
    with pytest.raises(DuplicateRegistrationError):
        instance_3.register_instance("instance_1")

    instance_2.register_instance("instance_2")
    instance_3.register_instance("instance_3")


def test_getting_registered_name_for_class() -> None:
    class Base(Choosable, registered_name="custom_base_name"):
        pass

    class Impl(Base):
        pass

    assert Base.get_registered_class_name() == "custom_base_name"
    assert Impl.get_registered_class_name() == "Impl"


def test_getting_registered_name_for_instance() -> None:
    class Base(Choosable):
        pass

    instance = Base()
    instance.register_instance("custom_instance_name")

    assert instance.get_registered_instance_name() == "custom_instance_name"
    assert Base.get_registered_class_name() == "Base"


def test_getting_registered_name_for_unregistered_instance() -> None:
    class Base(Choosable):
        pass

    unregistered_instance = Base()
    with pytest.raises(UnregisteredChoosableError):
        unregistered_instance.get_registered_instance_name()
