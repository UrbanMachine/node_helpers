Sensors
=======

The `node_helpers.sensors` module is designed to standardize the way sensor data is handled in ROS2. By providing reusable publishers, buffers, and visualization tools, it streamlines the process of creating new sensors and integrating them into your application.

Overview
--------

This module focuses on:

1. **Publishers**: A structured way to publish sensor messages, ensuring consistent QoS settings and optional visualization support.
2. **Buffers**: Simple tools for holding and retrieving the latest sensor data.
3. **Predefined Sensors**: Ready-to-use components for common sensor types like binary signals and rangefinders.

Core Components
---------------

### 1. BaseSensorPublisher

The `BaseSensorPublisher` class provides a base for publishing sensor messages with the following features:

- QoS settings optimized for sensor data (`qos_profile_sensor_data` by default).
- Support for RViz visualization via the `to_rviz_msg` method.
- Built-in throttling options for both sensor and visualization publishing rates.

Example:

.. code-block:: python

    from node_helpers.sensors.base_publisher import BaseSensorPublisher
    from std_msgs.msg import Header

    class CustomSensor(BaseSensorPublisher):
        def __init__(self, node, parameters):
            super().__init__(node, msg_type=CustomMsg, parameters=parameters)

        def to_rviz_msg(self, msg):
            # Implement visualization markers
            pass

### 2. BaseSensorBuffer

The `BaseSensorBuffer` class provides a simple interface for subscribing to sensor topics and accessing the latest readings. It includes:

- Event-based subscriptions via `on_value_change` and `on_receive`.
- Filtering of out-of-order messages.

Example:

.. code-block:: python

    from node_helpers.sensors.base_buffer import BaseSensorBuffer

    buffer = BaseSensorBuffer(node, CustomMsg, "/sensor/topic")
    latest_reading = buffer.get()

### 3. Predefined Sensors

- **Binary Sensors**: Tools for creating sensors with binary outputs (e.g., on/off).
- **Rangefinders**: Components for publishing rangefinder data with visualization support.

Binary Sensor Example:

.. code-block:: python

    from node_helpers.sensors.binary_signal import BinarySensor

    binary_sensor = BinarySensor(node, parameters)
    binary_sensor.publish_value(True)

Rangefinder Example:

.. code-block:: python

    from node_helpers.sensors.rangefinder import RangefinderPublisher

    rangefinder = RangefinderPublisher(node, parameters, qos)
    rangefinder.publish_range(1.5)  # Publish range in meters

