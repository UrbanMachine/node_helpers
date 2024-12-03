# node_helpers.launching

The `node_helpers.launching` module provides utilities to simplify ROS launch file management, including dynamic node swapping, and file validation. 

It supports the swapping of "real" and "mock" nodes during launch via the `SwappableNode` class and `apply_node_swaps` function. Additionally, it includes file validation utilities (`required_file` and `required_directory`) and URDF manipulation functions for namespace application and name validation.

Full documentation can be found under [docs/](../../../../docs/launching.rst).
