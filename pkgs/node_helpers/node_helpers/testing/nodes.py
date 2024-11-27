import contextlib
import logging
from collections.abc import Callable, Generator
from functools import partial
from multiprocessing import Process
from pathlib import Path
from queue import Queue
from threading import Thread
from time import sleep
from typing import TypeVar

import psutil
import rclpy
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node as LaunchNode
from launch_ros.parameter_descriptions import Parameter as LaunchParameter
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_services_default
from rclpy.task import Future

from node_helpers.nodes import HelpfulNode
from node_helpers.parameters import Namespace, ParameterLoader
from node_helpers.spinning import (
    DEFAULT_MULTITHREADED_THREAD_COUNT,
    MultiThreadedStackTracedExecutor,
)
from node_helpers.testing import each_test_setup_teardown  # noqa: F401
from node_helpers.timing import TestingTimeout as Timeout

MESSAGE_TYPE = TypeVar("MESSAGE_TYPE")


class NodeForTesting(HelpfulNode):
    """A ROS node, augmented with methods that are useful for testing"""

    def create_queue_subscription(
        self,
        type_: type[MESSAGE_TYPE],
        topic: str,
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
    ) -> "Queue[MESSAGE_TYPE]":
        """Subscribes to a topic, putting the results in a queue.

        :param type_: The type of message this topic uses
        :param topic: The topic to subscribe to
        :param qos_profile: The QoS profile to use for the subscription
        :return: The queue where values will be put
        """
        messages: "Queue[MESSAGE_TYPE]" = Queue()

        self.create_subscription(
            type_,
            topic,
            messages.put,
            qos_profile=qos_profile,
        )

        return messages

    def create_subscription_future(
        self,
        msg_type: type[MESSAGE_TYPE],
        topic: str,
        qos_profile: QoSProfile = qos_profile_services_default,
    ) -> Future:
        """Returns a future that finishes when a message is received on a topic"""
        future = Future()

        self.create_subscription(
            msg_type=msg_type,
            topic=topic,
            qos_profile=qos_profile,
            callback=future.set_result,
        )

        return future


NODE_CLASS = TypeVar("NODE_CLASS", bound=Node)

_INITIALIZED_TIMEOUT = 5


@contextlib.contextmanager
def rclpy_context() -> Generator[Context, None, None]:
    """A context manager for initializing a ROS context then cleaning up afterward"""
    context = Context()
    rclpy.init(context=context)

    try:
        yield context
    finally:
        rclpy.shutdown(context=context, uninstall_handlers=True)


def set_up_node(
    node_class: type[NODE_CLASS],
    namespace: str,
    node_name: str,
    *,
    default_params_directory: Path | str | None = None,
    parameter_overrides: list[Parameter] | None = None,
    remappings: dict[str, str] | None = None,
    multi_threaded: bool = False,
) -> Generator[NODE_CLASS, None, None]:
    """Sets up the given node class in a new ROS context with its own executor.
    Doing it this way better simulates runtime behavior, where each node is its own
    separate process. It also prevents some types of deadlocks that would not exist
    outside of testing.

    :param node_class: The class of the node to set up
    :param namespace: A namespace to put the node in
    :param node_name: The name to give the node
    :param default_params_directory: A directory to load parameters from. This is used
        when it's convenient to load parameters from yaml files, instead of specifying
        them in code. Yaml files will be loaded and overrided in alphanumeric order.
        Only the parameters for the given {namespace}.{node_name} will be loaded.
    :param parameter_overrides: Parameters values to provide to the node, overriding
        their values in configuration files
    :param remappings: Remappings for topics and other names, where the key is the
        name to be remapped, and the value is the new name
    :param multi_threaded: If True, the node will be started with a multi-threaded
        executor
    :yields: A node instance, ready to go
    """

    namespace_obj = Namespace([])
    namespace_obj += Namespace.from_string(namespace)
    namespace_obj += Namespace.from_string(node_name)

    # Optionally load parameters from a directory
    params = []
    if default_params_directory is not None:
        base_config_path = _find_path_in_parents(default_params_directory)
        params = ParameterLoader(
            parameters_directory=base_config_path
        ).parameters_for_node(namespace_obj)

    if parameter_overrides is not None:
        params = _merge_parameters(params, parameter_overrides)

    if remappings is None:
        remappings = {}

    cli_args = ["--ros-args"]
    for map_from, map_to in remappings.items():
        cli_args += ["--remap", f"{map_from}:={map_to}"]

    with rclpy_context() as context:
        if multi_threaded:
            executor = MultiThreadedStackTracedExecutor(
                context=context, num_threads=DEFAULT_MULTITHREADED_THREAD_COUNT
            )
        else:
            executor = SingleThreadedExecutor(context=context)

        node = node_class(
            context=context,
            namespace=namespace,
            parameter_overrides=params,
            cli_args=cli_args,
        )

        def spin() -> None:
            try:
                rclpy.spin(node, executor)
            except ExternalShutdownException:
                pass  # This exception always happens when shutting down the executor

        spin_node_thread = Thread(
            target=spin,
            name=f"Node Spin {node_class.__name__}",
            daemon=True,
        )
        spin_node_thread.start()

        # Wait for rclpy.spin to set the executor
        while node.executor is None:
            sleep(0.01)

        yield node

    # Explicitly destroy the node, instead of letting garbage collection do it later.
    # This ensures that node-specific cleanup operations happen in advance of other
    # shutdown procedures.
    node.destroy_node()
    assert executor.shutdown(timeout_sec=30), "Failed to shut down executor!"
    spin_node_thread.join(timeout=30)
    assert not spin_node_thread.is_alive()


def set_up_external_node(
    package_name: str,
    executable: str,
    *,
    is_ready: Callable[[Process], bool] | None = None,
    ready_timeout: float = _INITIALIZED_TIMEOUT,
    parameters: list[Parameter] | None = None,
    remappings: dict[str, str] | None = None,
    namespace: str | None = None,
) -> Generator[Process, None, None]:
    """Starts a node that is defined outside our codebase.

    :param package_name: The package the namespace is provided in
    :param executable: The name of the node executable
    :param is_ready: A function that can be called to check if the node is ready
    :param ready_timeout: The maximum amount of time to wait on the node to be ready
    :param parameters: Parameter values to assign to the node
    :param remappings: Remappings for topics and other names, where the key is the
        name to be remapped, and the value is the new name
    :param namespace: A namespace to move the node to. If this is not provided,
        the node is launched in the root namespace. This namespace must be prefixed
        with a forward slash, indicating that it is relative to the root namespace.
    :yields: The running process
    """

    # Reformat arguments to match the weird formats launch_ros uses
    remappings_launch = None
    if remappings is not None:
        remappings_launch = ((k, v) for k, v in remappings.items())
    parameters_launch = None
    if parameters is not None:
        parameters_launch = [LaunchParameter(p.name, p.value) for p in parameters]

    launch_node = LaunchNode(
        package=package_name,
        executable=executable,
        namespace=namespace,
        remappings=remappings_launch,
        parameters=parameters_launch,
    )
    yield from set_up_external_nodes_from_launchnode(
        [launch_node], is_ready=is_ready, ready_timeout=ready_timeout
    )


def set_up_external_nodes_from_launchnode(
    nodes: list[LaunchNode],
    is_ready: Callable[[Process], bool] | None = None,
    ready_timeout: float = _INITIALIZED_TIMEOUT,
) -> Generator[Process, None, None]:
    """Run an already prepared launch.Node in a new process"""

    launch_desc = LaunchDescription(nodes)
    launch_service = LaunchService()
    launch_service.include_launch_description(launch_desc)

    proc = Process(
        name=f"Launch Service for {nodes=}",
        target=partial(launch_service.run, shutdown_when_idle=False),
        daemon=True,
    )
    proc.start()

    if is_ready is not None:
        timeout = Timeout(
            ready_timeout,
            timeout_message=f"Node {nodes} took too long to become ready",
        )
        while not is_ready(proc) and timeout:
            sleep(0.01)

    yield proc

    # Ensure that the process did not exit early
    assert proc.is_alive()
    assert proc.pid is not None

    # Start killing all the children in the process group, wait, then kill the parent
    children_to_kill = psutil.Process(proc.pid).children(recursive=True)
    for child in children_to_kill:
        child.kill()

    for child in children_to_kill:
        try:
            child.wait(30)
        except Exception:
            logging.exception("Child process failed to exit")

        assert not child.is_running()

    proc.kill()

    # Block until the process exits
    proc.join(10)
    assert not proc.is_alive()
    assert proc.exitcode is not None
    proc.close()


def _find_path_in_parents(path: Path | str) -> Path:
    """Travels up the directory tree until the provided path can be found"""
    for parent in Path.cwd().parents:
        if (parent / path).exists():
            return parent / path

    raise RuntimeError(
        f"Could not find {path} in {Path.cwd()} or any of its parent folders"
    )


def _parameter_to_string(param: Parameter) -> str:
    if isinstance(param.value, list):
        # Lists take on the format [val1,val2,val3]
        output = "["
        values_str = [str(v) for v in param.value]
        if param.type_ is Parameter.Type.STRING_ARRAY:
            # Wrap the string in quotes to avoid parsing issues
            values_str = [f"'{v}'" for v in values_str]
        output += ",".join(values_str)
        output += "]"
    elif isinstance(param.value, str):
        # Wrap the string in quotes to avoid parsing issues if the string has special
        # characters in it
        output = f"'{param.value}'"
    else:
        output = str(param.value)

    return output


def _merge_parameters(
    base: list[Parameter], override: list[Parameter]
) -> list[Parameter]:
    """Combines the two lists of parameters, using the values from the second list in
    the case of parameter name collisions

    :param base: The base parameter list
    :param override: The parameters to apply over the base list
    :return: Combined parameters
    """

    base_dict = {v.name: v for v in base}
    override_dict = {v.name: v for v in override}
    base_dict.update(override_dict)
    return list(base_dict.values())
