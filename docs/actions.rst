Actions
===============

The ``node_helpers.actions`` framework is designed to provide an efficient and
easy-to-use structure for implementing and running actions in a ROS2 node. The purpose
is to facilitate DRY code, and the development of safe and robust action servers.

Implementing Actions
--------------------

The two key API's exposed by ``node_helpers.actions`` that assist with the
implementation of actions are the ``ActionWorker`` and ``ActionHandler``.
All actions in the Urban Machine codebase are written using these API's.

Let's start by reviewing two actions: one written the way an ordinary ROS action is
written, and another written using the ``node_helpers.actions`` framework.
Then, the differences between the two will be explained.

Comparing Ordinary Actions to Node Helpers
******************************************

Example Ordinary ROS Action
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    class CoolThingNode(Node):
        def __init__(self):
            super().__init__('cool_node')
            self._action_server = ActionServer(
                self,
                CoolThing,
                'cool_thing',
                self.action_callback
            )

        def action_callback(self, goal_handle: ServerGoalHandle) -> CoolThing.Result:
            try:
                self.get_logger().info('Executing goal...')

                while goal_not_reached:
                    # Publish some feedback
                    feedback_msg = CoolThing.Feedback()
                    goal_handle.publish_feedback(feedback_msg)

                    # Check for cancellation
                    if goal_handle.is_cancel_requested:
                        put_robot_in_safe_state()
                        goal_handle.canceled()
                        result = CoolThing.Result()
                        return result

                    # Make the robot do something
                    robot_do_thing()

                goal_handle.succeed()

                result = CoolThing.Result()
                return result
            except Exception as e:
                put_robot_in_safe_state()
                goal_handle.abort()
                raise e

Example node_helpers Action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    class CoolThingWorker(
        ActionWorker[CoolThing.Goal, CoolThing.Feedback, CoolThing.Result]
    ):
        def __init__(
            self, handler: "CoolThingActionHandler", goal_handle: ServerGoalHandle, logger: Logger,
        ) -> None:
            super().__init__(goal_handle, logger)
            self.handler = handler

        def run(self) -> Generator[CoolThing.Feedback | None, None, None]:
            """Action callback code goes here"""
            self.get_logger().info('Executing goal...')

            while goal_not_reached:
                # Yield some feedback (optional)
                yield CoolThing.Feedback()

                # Yield `None` during times when it's okay for the action to cancel.
                # The ActionWorker will automatically check for and handle cancellation.
                yield None

                # Make the robot do something
                robot_do_thing()

            # Set the result
            self.result = CoolThing.Result(was_cool=True)

        def on_exception(self, ex: Exception) -> None:
            """What to do if an unexpected exception happens."""
            put_robot_in_safe_state()

        def on_cancel(self) -> CoolThing.Result:
            """Cancelation code goes here"""
            put_robot_in_safe_state()

            return CoolThing.Result()

    class CoolThingActionHandler(
        FailFastActionHandler[CoolThing.Goal, CoolThing.Feedback, CoolThing.Result]
    ):
        class Parameters(BaseModel):
            some_parameter: float

        def __init__(self, node: HelpfulNode, action_name: str):
            super().__init__(node=node, action_name=action_name, action_type=CoolThing)
            self._params = node.declare_from_pydantic_model(self.Parameters, action_name)

        def create_worker(self, goal_handle: ServerGoalHandle) -> CoolThingWorker:
            return CoolThingWorker(handler=self, goal_handle=goal_handle, logger=self.node.get_logger())

Anatomy of the Example
**********************

CoolThingWorker is a subclass of ActionWorker and is responsible for executing the
action logic through its run method. The run method returns a generator that can yield
feedback messages (to be published) or None (to facilitate cancellation handling).

In case of cancellation, the ``on_cancel`` method is called to handle it and safely transition
the robot out of the current action. If the ``run`` method raises an exception, the on_exception method is invoked.

``CoolThingActionHandler`` is a subclass of ``FailFastActionHandler``. This means that it will
reject new action requests if the action is already in progress. The handler is responsible
for managing the action's parameters through a Pydantic Parameters class and creating
``CoolThingWorker`` instances to handle incoming action requests.

The create_worker method in ``CoolThingActionHandler`` constructs a new ``CoolThingWorker``
instance, providing the required parameters (handler, goal_handle, logger) for the execution
of the action.

Key Differences
****************

Here are the key differences between the two examples:

1. Clearer separation of concerns: node_helpers actions split the action logic out of
   nodes, and into separate files, encouraging development of reusable actions for multiple nodes.
2. Metrics. Each action responded to by a ``ActionHandler`` will publish metrics to a
   ``/metrics/write`` topic, which we use for visualizing actions in our ``Grafana`` dashboard.
3. Easier cancelation handling: node_helpers actions automatically handle cancelation
   requests by using a generator, which simplifies the cancelation logic.
4. Easier error handling: node_helpers actions will always run ``on_exception`` in case
   of an exception, reducing boilerplate.
5. Easily used alongside the ``node_helpers.parameters`` framework, allowing you to
   request per-action configuration in the ``ActionHandler``, and then use it in the
   ``ActionWorker``.
6. Takes care of calling the ``succeed``, ``abort``, and ``canceled`` methods of ``goal_handle``


Other Action Handlers
---------------------

- ``ActionHandler`` the default base handler will take care of creating the ``ActionServer``,
  and publishing metrics about the actions.
- ``FailFastActionHandler`` is useful for actions that the user knows should not run concurrently.
  For example, if you have an action that moves an axis on a robot, it might be bad if two callers
  were calling it at the same time. The ``FailFastActionHandler`` will quickly raise an exception,
  and give the writer of the action peace of mind that it can't be used incorrectly.
- ``QueuedActionHandler`` is useful for actions that aren't safe to run in parallel, but are
  safe to run sequentially in any order. For example, an action that takes a picture with a
  camera. This handler will queue up requests and respond to them in order.
- ``ContextActionHandler`` is useful for defining actions that behave like python Context Managers.
  This won't be covered in this tutorial.

Running Actions
---------------

Running a Single Action
***********************

When running a single action, it's almost always preferred to use the default ROS methodology.
When running an action synchronously, almost always we use ``send_goal``:

.. code-block:: python

    action_client.send_goal(CoolAction.Goal())

When running an action asynchronously, almost always the pattern is to run ``send_goal_async``,
wait for the goal to be accepted, then continue. Here's an example of **bad practice**:

.. code-block:: python

    send_goal_future = action_client.send_goal_async(CoolAction.Goal())
    # Wait for the goal to be accepted by the server
    while not send_goal_future.done():
        pass
    goal_handle = send_goal_future.result()

    # Get the future for the goal result
    result_future = goal_handle.get_result_async()

That's so much boilerplate! There is a shorthand for type of sequence this under ``node_helpers.futures``:

.. code-block:: python

    from node_helpers import futures

    result_future, client_handle = futures.wait_for_send_goal(
        action_client, CoolAction.Goal()
    )

In this example, the ``wait_for_send_goal`` waits for the goal to be accepted, then
returns the result future and the client handle.

Running Many Actions
********************

This robot has a lot of moving parts, and a lot of actions for those moving parts. A common
pattern is to send multiple actions in parallel to different parts of the robot, requesting
that those parts start moving at the same time.

There is a mini framework for making that sort of code easier to write. It consists of
``ActionElement``, ``ActionGroup``, and ``ActionSequence`` APIs. The most important is the
``ActionGroup``.

ActionGroup
~~~~~~~~~~~

An ``ActionGroup`` is a utility class for running multiple actions simultaneously and
waiting for their completion or a specified partial completion. It can be thought of
as a framework for combining multiple actions into a single object which you can use
as though it were one action.

It takes in a list of ``ActionElements``, which are just ``dataclasses`` that hold the
action client and the goal to run on it.

Creating an ActionGroup
^^^^^^^^^^^^^^^^^^^^^^^

Below is an example of an action group being created. No action is run in this example.

.. code-block:: python

    # Create an ActionGroup with ActionElements
    action_group = ActionGroup(
        ActionElement(client=action_client_1, goal=goal_1),
        ActionElement(client=action_client_2, goal=goal_1, feedback_callback=on_feedback),
    )

Running Synchronously
^^^^^^^^^^^^^^^^^^^^^

ActionGroups mirror the ROS ActionClient API, with some added features. Here's an example
of actually running the action group, and synchronously waiting for the results:

.. code-block:: python

    results = action_group.send_goals()

Running Asynchronously
^^^^^^^^^^^^^^^^^^^^^^

It's also desireable sometimes to run multiple actions, and not block. Here's how you
run all the actions asynchronously, and then choose when you block for results.

.. code-block:: python

    # The following line starts all of the actions in the group in parallel
    action_group.send_goals_async()

    # You can blockingly wait for results by using:
    results = action_group.wait_for_results()

    # Alternatively, you can yield periodically until results arrive. This gives
    # ActionWorkers a chance to check for cancellation, if running this action group
    # within the context of a larger action
    results = yield from action_group.yield_for_results()

Cancelling Action Groups
^^^^^^^^^^^^^^^^^^^^^^^^

It's also possible to request cancellation of an entire action group at once:

.. code-block:: python

    action_group.cancel_goals()