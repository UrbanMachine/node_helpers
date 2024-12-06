# node_helpers.urdfs

The `node_helpers.urdfs` module provides tools to streamline working with URDFs in ROS2, enabling efficient URDF handling for launch files, testing, and runtime validation.

### Features
- **URDFConstant**: Facilitates URDF validation and consistent referencing of joints and frames in code.

There are other URDF utilities in `node_helpers` in other modules, such as
- `node_helpers.launching.URDFModuleNodeFactory`: Automates the creation of launch file nodes for URDFs, including robot and joint state publishers.
- `node_helpers.testing.URDFModuleFixture`: Simplifies URDF testing by providing a fixture for loading URDFs and creating robot state publishers.
- `Helper Functions`: Additional utilities for namespace application, path fixing, and frame validation.

For detailed usage, examples, and API reference, see the [comprehensive documentation](../../../../docs/urdfs.rst).
