# node_helpers.sensors

The `node_helpers.sensors` module provides a standardized framework for handling sensor data in ROS2. It includes tools for publishing, buffering, and visualizing sensor messages, making it easy to implement and reuse components across different sensor types.

Key features:
- **Publishers**: `BaseSensorPublisher` facilitates structured sensor message publishing with built-in support for RViz visualization.
- **Buffers**: `BaseSensorBuffer` holds and updates the latest sensor readings for on-demand access.
- **Predefined Sensors**: Includes reusable components for binary sensors and rangefinders.

This framework simplifies creating and visualizing new sensors while promoting best practices for consistent sensor message handling.

For detailed documentation, see [docs/](../../../../docs/sensors.rst)
