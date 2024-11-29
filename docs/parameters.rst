Parameters
===============

The parameters framework in our robot is designed to make managing and updating parameters in your ROS nodes efficient
and easy. This documentation will guide you through the process of defining parameters in your ROS nodes and managing
them using the layered parameter system.

Why Not Use ROS Parameters Directly
-----------------------------------

The Parameters framework uses ROS parameters behind the scene. Why are we using a framework in the first place?

1) **Less boilerplate**: Simplify the process of declaring and getting parameters, reducing the
   amount of boilerplate code you need to write when working with parameters.

2) **Pydantic model integration**: Allow you to define parameters using Pydantic models, which provides
   automatic validation and type checking for your parameters. This helps ensure that the parameters you define are
   correct and adhere to the specified constraints.

3) **Custom type parsing support**: The Parameters library has various custom types that you can use
   to automatically convert from the type in the Yaml file to a specific python type. Want to parse that
   string as a Path? Or as a specific Enum? No problem!

4) **Required parameters**: Enable you to mark certain parameters as required, meaning that they must be
   set from an external source (such as a parameter file). If a required parameter is not set, an
   exception is raised, allowing you to detect and handle missing parameter values more
   effectively.

5) **Subscribe to parameter updates**: Offers a way to subscribe attributes to parameter updates. When a
   parameter is updated, the corresponding attribute is automatically updated as well, making it easier to keep your
   robot's state in sync with the parameter values.


Usage Examples
--------------

Basic Declaration of Pydantic Parameters
****************************************

Here's an example of using the ``Parameters`` framework within a ROS node.

First off, you'll want to be working with a node that subclasses the ``ParameterMixin``. Typically in a codebase, the
``node_helpers.nodes.HelpfulNode`` will be used, because it provides all of the ``node_helper`` mixins at once, including
the ``ParameterMixin``.

..  code-block:: python

    class CoolNode(HelpfulNode):
        class Parameters(BaseModel):
            wood_tf: str

            some_list_of_numbers: list[int]

            calibration_thing: float

            my_path: Path

            some_enum: MyCustomEnum

            maybe_int_maybe_str: int | str

            value_or_default_none: MyCustomEnum | None = None

        def __init__(self, **kwargs: Any) -> None:
            super().__init__("cool_node", **kwargs)
            self.params = self.declare_from_pydantic_model(self.Parameters, "node")

In the above example, if launched, the ``self.params`` variable will have all of the loaded parameters from the
file, type checked, validated, and required to be filled in. If the file doesn't have those parameters specified, it
will raise an exception and the runner of the stack will have a clear error message.

Furthermore, these parameters can be changed live, during runtime, using tools like ``RQT``, because the ``ParameterMixin``
automatically subscribes pydantic model attributes to changes in their respective parameters. For example, the
``params.calibration_thing`` attribute could be edited during runtime.

The Override-This Value
^^^^^^^^^^^^^^^^^^^^^^^

There's special support for a sentinal value ``<override this>``. If you set a parameter
to this value, it will raise an exception if the parameter is not set in the configuration file.


The 'Choosable' System
****************************

The ``Parameters`` framework also supports a 'choosable class' system, which allows you
to specify a parameter that will be used to select between different types of classes (or 
instances of classes).

A common pattern in our codebase is to have different implementations of some Base Class
which are used in different situations. For example, we might have a ``BaseCamera`` class
that has different implementations for different types of cameras. We might have a
``RealCamera`` class and a ``SimulatedCamera`` class, and we want to be able to choose
between them using a parameter.

To do this, we can have the ``BaseCamera`` class inherit from
``Choosable`` type, which allows you to specify any cameras in pydantic
models, have have the type automatically found and returned.

Basic Use
^^^^^^^^^

.. code-block:: python

    # Declaration of registered classes
    class BaseCamera(Choosable):
        # Normal base class stuff here
        ...

    class RealCamera(BaseCamera):
        # Camera implementation here
        ...

    class SimulatedCamera(BaseCamera):
        # Camera implementation here
        ...

    # Using a choosable class in a pydantic model
    class Parameters(BaseModel):
        camera_type: type[BaseCamera]  # <-- notice how this is wrapped with type[]

    # In a node somewhere
    self.params = self.declare_from_pydantic_model(self.Parameters, "camera_params")

Then in configuration, to set the value of camera_type, you can use the following:

.. code-block:: yaml

    camera_params:
        camera_type: "RealCamera"

This type can then be accessed via ``self.params.camera_type``.

Custom Names
^^^^^^^^^^^^

It can be desireable to have a different name for the parameter than the name of the
class. For example, we might want to have a camera be referred to as 'real_camera'
but the class name is 'RealCamera'. To do this, we could change the above example and
use metaclass parameters to specify the ``registered_name`` argument:

.. code-block:: python

    class RealCamera(BaseCamera, registered_name="real_camera"):
        pass

Now in configuration this can be accessed like such:

.. code-block:: yaml

    camera_params:
        camera_type: "real_camera"

Choosable Instances
^^^^^^^^^^^^^^^^^^^

Just like you can have choosable classes, you can also have choosable instances. This
allows you to 'register' instantiated objects and have them be accessible via a
parameter.

This is most used in the ``urdf_data.constants`` package, where we have many different
URDFConstants instances that are referenced by name in the configuration.

Here's an example of a Choosable Instance:

.. code-block:: python

    class MyChoosableInstance(Choosable):
        ...

    my_instance_a = MyChoosableInstance(cool_param=3)
    my_instance_a.register_instance("instance_1")

    my_instance_b = MyChoosableInstance(cool_param=5)
    my_instance_b.register_instance("instance_2")

    class Parameters(BaseModel):
        my_instance: MyChoosableInstance  # <-- notice how this isn't wrapped with type[]

    # In a node somewhere
    self.params = self.declare_from_pydantic_model(self.Parameters, "my_params")

Then in configuration, to set the value of my_instance, you can use the following:

.. code-block:: yaml

    my_params:
        my_instance: "instance_1"

.. warning::

    When registering an instance, this will hold a reference to the instance in memory!
    It will never get garbage collected.

    In the future, we may want to change this feature to instead use weakrefs.



Parameter Loading and Layered Parameters
----------------------------------------

Our parameter system is designed to be layered, which means it can load multiple YAML files and combine them. This
allows you to have different parameter configurations for different environments or situations, and easily switch
between them.

The ``ParameterLoader`` is responsible for loading the parameter files and merging them together. When writing a launch
file, you can specify which parameter files should be loaded, and the ``ParameterLoader`` will take care of merging them
in the order specified.

For information about how the parameters are configured for this robot, check out our `Configuration`_ docs

.. _Configuration: ../../deployment/ros_configuration.html

Meta Parameters
***************

In addition to the standard ROS parameters, our ``ParameterLoader`` system also supports an optional ``MetaParameters``
field to be specified. ``MetaParameters`` allow you to insert a custom Pydantic model into an otherwise fully
ROS-parameter compatible YAML file.

Why do this? Well, it's tremendously helpful when writing launch files to have configuration
for which nodes `the launch file will create` in the same place as the actual ROS parameter
configuration for the nodes themselves!

For example, you might have a configuration file that looks like this:

.. code-block:: yaml

    meta_parameters:
        camera_namespaces: ["camera_1", "camera_2"]
        urdfs: ["/path/to/urdf", "/path/to/another/urdf"]

    camera_1:
        camera_node:
            some_param: 3

    camera_2:
        camera_node:
            some_param: 5

Then inside of the launch file you specify the MetaParameters using the following model:

.. code-block:: python

    class MyCoolMetaParameters(BaseModel):
        camera_namespace: list[str]
        urdfs: list[str]

Using this, you can use ``MyCoolMetaParameters`` when loading parameters using the ``ParameterLoader``, and extract
information from the configuration in order to dynamically generate all of the nodes specified under ``camera_namespaces``.

