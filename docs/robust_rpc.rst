Robust RPC
===============

The RobustRPC framework is one of the key components of ``node_helpers``. Its key API
is the ``RobustRPCMixin``, which provides a robust approach to handling errors in service
and action calls by propagating error messages raised by the server and re-raising them
on the client side.
This documentation aims to help users understand and effectively use the RobustRPCMixin.


What is it, and why use it?
---------------------------
The RobustRPCMixin is a mixin class in the node_helpers ROS package designed to simplify error handling for remote procedure calls using services and actions. By integrating this mixin into your custom ROS nodes, you can improve error handling and maintainability in your robotic system.

Key benefits of using RobustRPCMixin:

- Automatically propagate error messages from the server to the client
- Pythonic error handling- you can 'try/except' remote errors
- Reduce boilerplate code (no more checking if every action succeeded or failed)

Main Classes and Features
-------------------------

1. ``RobustRPCMixin``: This mixin adds methods for creating service and action clients
   and servers. It ensures that any errors raised by the server are caught, passed in
   the response message, and re-raised on the client side. The client will raise an
   error of the same name that subclasses ``RobustRPCException``.

2. ``RobustActionClient``: A subclass of the ActionClient that wraps it to provide
   functionality for raising remote errors when calling ``send_goal_async().result()`` or
   ``send_goal()``. It includes the ``send_goal_as_context`` method, which is a context
   manager that sends a goal and cancels it after the context ends.

3. ``RobustServiceClient``: A subclass of the rclpy.Client that wraps it to provide
   functionality for raising remote errors when calling call_async().result() or call().

4. Both the action and service clients will **automatically wait until the server is online!**
   This means that upon the first call, they will wait until the server is online before
   actually making the call. This fixes the annoying default ROS2 behavior of allowing
   you to make the call, then quietly hanging and blocking forever.


What do I have to change to use it?
-----------------------------------

Only two things!

1. First, when instantiating a service or action, use the ``RobustRPCMixin`` methods
   to do so.

2. Use messages that contain the following:

   .. code-block:: text

        string error_name
        string error_description

   That is to say, in an Action message, include those two fields in the ``result`` message.
   In a service message, include those two fields in the ``response`` message.

Server Usage Example
--------------------

In the example below, the ``YourNode`` class is a subclass of the ``Node`` and
``RobustRPCMixin`` classes. Both service and action servers are created using the
``create_robust_service`` and ``create_robust_action_server`` methods from the ``RobustRPCMixin``.

The callback functions for the service and action server raise
ordinary python exceptions. When the exception is raised, it is magically captured by
the RobustRPCMixin wrappers, which pack the error name and description into the message,
and send that message back to the client.

The robust client will then re-raise those exact exceptions.

.. code-block:: python

    from rclpy.node import Node
    from node_helpers.robust_rpc import RobustRPCMixin, RobustRPCException
    from your_package.msg import YourRobustAction, YourRobustService


    class YourNode(Node, RobustRPCMixin):
        def __init__(self):
            super().__init__("your_node")

            # Create robust service server
            self.service_server = self.create_robust_service(
                srv_type=YourRobustService,
                srv_name="your_service_name",
                callback=self.service_callback
            )

            # Create robust action server
            self.action_server = self.create_robust_action_server(
                action_type=YourRobustAction,
                action_name="your_action_name",
                execute_callback=self.action_callback
            )

        def service_callback(self, request: YourRobustService.Request, response: YourRobustService.Response):
            # This behaves like an ordinary service callback, except that it's okay to
            # raise python exceptions within it.
            if something_bad:
                raise SomeSpecificError("Oh no, something bad happened in my service!")
            return response

        def action_callback(self, goal_handle: ServerGoalHandle):
            # This behaves like an ordinary action callback, except that it's okay to
            # raise python exceptions within it.
            if something_bad:
                raise SomeSpecificError("Oh no, something bad happened in my action!")

            goal_handle.succeed()
            return result

Client Usage Example
--------------------

In the example below, the client-side of the robust framework is demonstrated.
By using the ``create_robust_client()`` and ``create_robust_action_client()`` methods,
any exceptions raised in the server side will be re-raised on the client side.

Furthermore, the client will automatically wait for the server to come online on the
first time it's called. On subsequent calls, this check will be skipped.

The ``call_service()`` and ``call_action()`` methods show how to call a service and
call an action using these clients, with proper error handling using the
``RobustRPCException.like()`` method and the ``RobustRPCException`` class.

.. code-block:: python

    from rclpy.node import Node
    from node_helpers.robust_rpc import RobustRPCMixin
    from your_package.msg import YourRobustAction, YourRobustService


    class YourNode(Node, RobustRPCMixin):
        def __init__(self):
            super().__init__("your_node")

            # Create robust service client
            self.service_client = self.create_robust_client(
                srv_type=YourRobustService, srv_name="your_service_name"
            )

            # Create robust action client
            self.action_client = self.create_robust_action_client(
                action_type=YourRobustAction, action_name="your_action_name"
            )

        def call_service(self, request: YourRobustService.Request):
            try:
                response = self.service_client.call(request)
                # Process response
            except RobustRPCException.like("SomeSpecificError"):
                # Handle specific error
            except RobustRPCException as e:
                # Handle all other robust RPC errors

        def call_action(self, goal: YourRobustAction.Goal):

            # Here is an example of the context-manager API of the action client.
            # Upon entering the context, the action will be executed. Upon exiting the
            # context, the action will be cancelled.
            with self.action_client.send_goal_as_context(goal) as goal_handle:
                # Perform tasks while goal is being executed
                # ...
                # The context manager will automatically cancel the goal if not finished

            # Here is an example of calling an action that can might raise exceptions
            # remotely, and how to catch and handle those exceptions
            try:
                result_future = self.action_client.send_goal(goal)
                # Process result
            except RobustRPCException.like(SomeSpecificError):
                # Handle this specific error type
            except RobustRPCException as e:
                # Handle all other robust RPC errors that are not "SomeSpecificError"


Catching Errors on the Client Side
**********************************

When using a robust service or action client, errors raised on the server side will be
caught, passed in the response message, and re-raised on the client side. Because the
client side is raising exceptions from strings, all client side errors will subclass
``RobustRPCException``. To catch these errors by "name", you can use the
``RobustRPCException.like`` method:

.. code-block:: python

    from node_helpers.robust_rpc import RobustRPCException

    try:
        response = robust_service_client.call(request)
    except RobustRPCException.like(ValueError):
        # Handle a "ValueError" that was raised on the server side
        ...
    except RobustRPCException:
        # Handle any other errors raised by the server
        ...

It is possible to use the `like()` method with a string value, instead of a reference
to an exception. This is typically bad practice, because the point of using a reference
is to aid when refactoring codebases, and discoverability of where that error might
be raised. If the exception gets renamed or removed, our lint tooling will catch the
error and alert the developer.