# node_helpers.pubsub

This module provides an in-process publish-subscribe (pub-sub) system akin to a signals/slots mechanism. While ROS itself is a pub-sub system for inter-process communication using topics, this module is designed for intra-process communication, enabling lightweight, event-driven interactions between components within the same process.

This approach is ideal for scenarios where you need responsive communication without the overhead of ROS's networked messaging system.


Here's where one might use a topic:

```python3
class ExampleSensorBuffer(ABC, Generic[SENSOR_MSG]):
    """Some sensor buffer"""

    def __init__(self, ...):
        super().__init__()
        # Topics
        self.on_value_change = Topic[SENSOR_MSG]()
        """Called when a reading has changed from a previous value"""
        self.on_receive = Topic[SENSOR_MSG]()
        """Called whenever a new reading is received"""
```

This could then be used by other components like so:

```python3

class ExampleSensor:
    """Some sensor"""

    def __init__(self, buffer: ExampleSensorBuffer):
        super().__init__()
        self.buffer = buffer
        self.buffer.on_value_change.subscribe(self.on_value_change)
        self.buffer.on_receive.subscribe(self.on_receive)

    def on_value_change(self, msg: SensorMsg):
        print(f"Value changed: {msg}")
        # Do something with the new value

    def on_receive(self, msg: SensorMsg):
        print(f"Received: {msg}")
        # Do something with the new value
```

The topics could be for any use, not just for sensors. They could be for any kind of event-driven communication within a process.