# node_helpers.destruction

This module provides a utility mixin, DestroyCallbacksMixin, to enhance ROS 2 nodes with a flexible destruction lifecycle. It enables developers to register custom callbacks that are invoked when the node is destroyed, ensuring proper resource cleanup and lifecycle management.

This mixin is particularly useful for managing resources like threads, file handles, or other external dependencies that need explicit teardown when a node is shut down.

