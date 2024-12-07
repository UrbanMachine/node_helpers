URDF Utilities
==============

There are several URDF tools in `node_helpers` for launching, validating, and testing URDF-based systems, providing a standardized approach to handling URDFs in robotics applications.

Overview
--------

The module includes the following components:
1. **node_helpers.launching.URDFModuleNodeFactory**: Streamlines creation of `joint_state_publisher` and `robot_state_publisher` for URDFs in launch files.
2. **node_helpers.urdfs.URDFConstant**: Provides consistent access to URDF frames and joints, with validation tools. This is key for accessing 'tf' frames and 'joints' in a standardized way, without having random string constants sprinkled around your codebase.
3. **node_helpers.testing.URDFModuleFixture**: Facilitates launching URDF modules for integration tests.

URDFConstant
------------

The `URDFConstants` class provides a structured way to reference and validate URDF elements, such as joints and frames. It ensures URDF correctness and avoids duplicate names or missing elements.

**Features**:

- Load multiple URDF's but refer to them as a single module in code.
- Prepend namespaces to avoid conflicts.
- Validate that joints and frames exist in the URDF.
- Dynamically adjust URDFs with namespaces.

**Example**:

Below, we create the concept of a "BigBird" robot, which consists of two URDFs.
We then, at the bottom, create a `BigBirdURDF` object that encapsulates the URDFs and provides access to the joints and frames.

The BigBirdJoint and BigBirdFrames classes define the joints and frames in the URDFs,
and refer to real URDF elements by their names, prepended with `bird_gantry` or `bird_base`
to point back to what URDF file they came from. The `urdf_paths` parameter in the `URDFConstants` constructor
specifies what URDF the prepended names refer to.

.. code-block:: python

    from typing import NamedTuple

    from urdf_data.urdf_constants import URDFConstants


    class BigBirdJoints(NamedTuple):
        X: str = "bird_gantry.xaxis"
        Y: str = "bird_gantry.yaxis"
        Z: str = "bird_gantry.zaxis"
        PAN: str = "bird_gantry.waxis"


    class BigBirdFrames(NamedTuple):
        BASE_LINK: str = "bird_base.gantry_base_link"

        X_AXIS_ORIGIN: str = "bird_gantry.xaxis_parent_datum"
        X_AXIS_CURRENT: str = "bird_gantry.gantry_xlink"

        Y_AXIS_ORIGIN: str = "bird_gantry.yaxis_parent_datum"
        Y_AXIS_CURRENT: str = "bird_gantry.gantry_ylink"

        Z_AXIS_ORIGIN: str = "bird_gantry.zaxis_parent_datum"
        Z_AXIS_CURRENT: str = "bird_gantry.gantry_zlink"

        PAN_ORIGIN: str = "bird_gantry.waxis_parent_datum"
        PAN_CURRENT: str = "bird_gantry.gantry_wlink"


        TOOL_TIP: str = "bird.grasp_point"


    BigBirdURDF = URDFConstants[BigBirdJoints, BigBirdFrames](
        registration_name="bird_robot",
        urdf_paths=[
            ("bird_base", "path/to/bird_base/robot.urdf"),
            ("bird_gantry", "path/to/bird_gantry/robot.urdf"),
        joints=BigBirdJoints(),
        frames=BigBirdFrames(),
    )

Note that an example URDF constant can be found in ``pkgs/node_helpers_test/integration/urdfs/example_urdf_constants.py``

URDFModule
----------

The `URDFModuleNodeFactory` simplifies launching URDF nodes by generating `robot_state_publisher` and `joint_state_publisher` nodes for each URDF file. It applies namespaces to avoid collisions and ensures URDFs are properly loaded and validated.

In the below example, a ``joint_state_publisher`` will be created under the ``/big_bird_left/`` namespace,
and multiple ``robot_state_publishers`` will be created for each URDF file in the `BigBirdURDF` constant.
For example, one will live under ``/big_bird_left/urdf_0/`` and the other under ``/big_bird_left/urdf_1/``.

They will all publish to the same ``joint_state_publisher`` under the ``/big_bird_left/`` namespace.

**Example**:

.. code-block:: python

    from node_helpers.urdfs.urdf_module_launching import URDFModuleNodeFactory

    parameters = URDFModuleNodeFactory.Parameters(
        namespace: "big_bird_left",
        urdf_constant_name: "BigBirdURDF",
        apply_namespace_to_urdf: True,
    )
    factory = URDFModuleNodeFactory(parameters)
    nodes = factory.create_nodes()  # these nodes can be added to a launch description


URDFModuleFixture
------------------

The ``URDFModuleFixture`` class is a pytest fixture utility for setting up URDF-based tests. It
will launch the URDF module (and all it's `robot_state_publisher`s and `joint_state_publisher`,
and ensure that all TF frames are published correctly before yielding the fixture.

**Example**:

.. code-block:: python

    from node_helpers.urdfs.urdf_module_fixture import URDFModuleFixture

    @pytest.fixture()
    def big_bird_urdf_module() -> Generator[URDFModuleFixture, None, None]:
        yield from URDFModuleFixture.set_up(
            URDFModuleNodeFactory.Parameters(
                namespace="big_bird_top", urdf_constant_name=BigBirdURDF.registration_name
            )
        )


A full example of how to integration test URDFs can be found under ``pkgs/node_helpers/node_helpers_test/integration/urdfs/test_forklift.py``

Note that ``node_helpers`` provides a helpful test URDF in ``pkgs/node_helpers/sample_urdfs/forklift/robot.urdf``