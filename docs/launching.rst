Launching
=========

The `node_helpers.launching` module provides utility functions and classes to streamline the management of ROS launch files.

Core Features
-------------

1. **Node Swapping**:
   - The `SwappableNode` class allows nodes to track their name and namespace explicitly and facilitates dynamic swapping between "real" and "mock" nodes using the `SwapConfiguration` model.
   - The `apply_node_swaps` function ensures a one-to-one mapping between nodes and their mocks, enforcing validation and consistency in swap configurations.

   **Example Usage**:

   .. code-block:: python

       from node_helpers import launching

       class MetaParameters:
           swaps: dict[str, launching.SwapConfiguration]

        param_loader: ParameterLoader[MetaParameters] = ParameterLoader(
            parameters_directory=Path("/robot/launch-profile/"),
            override_file=Path("/robot/launch-profile/config.override.yaml"),
            meta_parameters_schema=MetaParameters,
        )

        launch_description = [
           launching.SwappableNode(namespace="example_namespace", name="real_node"),
           launching.SwappableNode(namespace="example_namespace", name="mock_node"),
        ]

        filtered_launch = apply_node_swaps(param_loader.meta_parameters.swaps, launch_description)

   This loads parameters from yaml files in the launch-profile directory, applies the swap configuration to the launch description, and returns a filtered launch description with the correct nodes.

2. **File Validation**:
   - Utility functions such as `required_file` and `required_directory` verify the existence of critical files and directories before launching. These checks prevent runtime errors caused by missing resources.

   **Example Usage**:

   .. code-block:: python

       from node_helpers import launching

       config_file = launching.required_file("/path/to/config.yaml")

Error Handling and Validation
-----------------------------

The module includes robust error handling:

- `InvalidSwapConfiguration` is raised when node swapping configurations are inconsistent or incomplete.
- Validation ensures all specified nodes are present in the launch description and that pairs of swappable nodes are correctly configured.

