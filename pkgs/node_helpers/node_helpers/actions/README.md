# node_helpers.actions

This module provides a robust framework for creating and managing ROS 2 actions, centered around two key APIs: ActionWorker and ActionHandler. These classes form the foundation of the module, enabling developers to define custom action logic, handle feedback and cancellations, and integrate seamlessly with ROS's action server infrastructure.

**`ActionWorker`**:
An abstract base class for implementing action execution logic with built-in support for feedback, timeouts, and cancellations.


**`ActionHandler`**:
A configurable handler that manages an action server, delegates goals to ActionWorker instances, and provides hooks for metrics and custom behavior.
This module also includes tools for sequencing actions, managing complex workflows, and enforcing safe execution patterns, such as fail-fast behavior or context-managed long-running actions.


The framework is documented in [docs/](../../../../docs/actions.rst).