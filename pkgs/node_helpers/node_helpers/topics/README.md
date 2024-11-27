# node_helpers.topics

This module provides tools for working with ROS2 topics, including utilities that augment publishing and subscribing.

- **LatchingPublisher**: A publisher with behavior similar to ROS1's latching QoS. It ensures the latest message is consistently available to new subscribers by periodically republishing the last message.

These tools help ensure robust and reliable topic communication in ROS2 systems.
