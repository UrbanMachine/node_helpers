# node_helpers.timing

The `node_helpers.timing` module provides utilities for managing time in ROS2, including context-based timing and caching. It includes various tools tailored for specific use cases:

- **Timers**: Use `Timer` for profiling and `WarningTimer` to detect slow callbacks.
- **Mixins**: `SingleShotMixin` simplifies creating one-time timers, while `TimerWithWarningsMixin` logs warnings if timers fall behind.
- **Timeouts**: `Timeout` enforces time limits, and `TestingTimeout` is ideal for test cases.
- **TTL Caching**: The `ttl_cached` decorator enables caching method results for a set duration.

These tools streamline time management and enhance system reliability.
