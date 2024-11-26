# node_helpers.futures

This module provides utility functions to simplify working with `rclpy.Future` objects in ROS 2. These tools make it easier to manage asynchronous tasks, including waiting for completion, handling timeouts, preemption, and executing actions with a clean, synchronous-like interface.

The module is especially useful for scenarios where you need robust handling of ROS futures in a multi-threaded environment or want to integrate asynchronous workflows without adding complexity.


For example:
```python3

from node_helpers import futures

future = node.call_some_action()
result = futures.wait_for_future(future, ExpectedType, timeout=5.0)

# Or with multiple futures
results = wait_for_futures([future1, future2], ExpectedType, timeout=10.0)

```