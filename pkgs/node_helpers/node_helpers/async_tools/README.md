# node_helpers.async_tools

The node_helpers.async_tools module provides a utility class, AsyncAdapter, to bridge the gap between traditional ROS 2 nodes and asyncio-based asynchronous programming. This is particularly useful for integrating modern Python asynchronous networking and workflows into ROS 2 systems, which predominantly rely on callback-driven and multithreaded architectures.

## Key Features
**Background Event Loop**: Creates and manages a dedicated asyncio event loop that runs independently of the ROS event loop.
**Async Callback Adaptation**: Converts asyncio coroutines into functions compatible with ROS callback mechanisms, enabling seamless integration of async workflows.
**Clean Shutdown**: Ensures proper cleanup of the asyncio event loop during node destruction.