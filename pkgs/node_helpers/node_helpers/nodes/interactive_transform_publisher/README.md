# interactive_transform_publisher

The `interactive_transform_publisher` is a ROS2 node for interactively manipulating and publishing static transforms using RViz. It simplifies calibration and configuration of transforms for URDFs or static TF trees.

### Features
- **Interactive Editing**: Manipulate transforms visually in RViz using interactive markers.
- **Persistence**: Save and load transforms from a configuration file for reuse across sessions.
- **API for Adjustments**: Modify and create transforms programmatically using provided topics and client utilities.

### Key Topics
- `/tf_static_updates`: Update existing transforms.
- `/tf_static_create`: Create new transforms dynamically.

For full documentation, see [interactive_transform_publisher.rst](../../../../docs/interactive_transform_publisher.rst).
