from collections import defaultdict
from collections.abc import Iterable
from typing import Any

from launch import LaunchDescriptionEntity
from pydantic import BaseModel
from rclpy.node import Node


class InvalidSwapConfiguration(Exception):
    pass


class SwapConfiguration(BaseModel):
    mock: str
    real: str
    enable_mock: bool


class SwappableNode(Node):
    """Unfortunately there's no way to know what the 'name' or 'namespace' of a node is
    by looking at the Node.name or Node.namespace. So we subclass node and keep track
    of those attributes in the __init__

    If this node exits early during operation, the entire system will shut down.
    """

    def __init__(self, *, namespace: str, node_name: str, **kwargs: Any):
        super().__init__(namespace=namespace, node_name=node_name, **kwargs)
        self.swap_namespace = namespace
        self.swap_name = node_name

    def should_use(self, configuration: SwapConfiguration) -> bool:
        if configuration.real == self.swap_name:
            if configuration.enable_mock:
                return False
            return True
        elif configuration.mock == self.swap_name:
            if configuration.enable_mock:
                return True
            return False
        else:
            raise InvalidSwapConfiguration(
                "This function should only be fed relevant SwapConfiguration. "
                f"Received {configuration=} for {self.swap_name=}"
            )


def apply_node_swaps(
    configuration: dict[str, SwapConfiguration],
    launch_description: Iterable[LaunchDescriptionEntity],
) -> list[LaunchDescriptionEntity]:
    """This system is used to swap "node A" or "node B" using configuration. This is
    most useful when there's a 1:1 correspondence between nodes and their respective
    mocks.

    :param configuration: A dictionary of {'namespace': MockConfiguration}
    :param launch_description: Output from a normal generate_launch_description
    :raises InvalidSwapConfiguration: If theres any incorrect configuration either in
        the swap configuration, or if nodes are missing that should have existed.
    :return: A list of filtered launch entities, with swaps applied
    """
    filtered_launch_description = []
    seen_referenced_nodes: dict[str, list[str]] = defaultdict(list)
    """Keep track of all referenced namespace:node_name pairs in the configuration.
    If any aren't fully described after looking through the launch description, raise
    an exception.
    """

    for element in launch_description:
        if not isinstance(element, SwappableNode):
            filtered_launch_description.append(element)
            continue

        # Track the nodes name
        seen_referenced_nodes[element.swap_namespace].append(element.swap_name)

        # Validate configuration contains the name
        if element.swap_namespace not in configuration:
            raise InvalidSwapConfiguration(
                f"Expected swap configuration for "
                f"{element.swap_namespace}.{element.swap_name} but was given none! "
                f"{configuration=}"
            )

        # Decide if the node should be kept
        swap_configuration = configuration[element.swap_namespace]
        if element.should_use(swap_configuration):
            filtered_launch_description.append(element)

    # Validate all nodes that were referenced in configuration were seen
    if len(seen_referenced_nodes.keys()) != len(configuration.keys()):
        raise InvalidSwapConfiguration(
            "Not all namespaces referenced in the swap configuration were seen in the "
            "launch file description!"
        )
    # Validate all seen namespaces contain two nodes that would have been swapped
    for seen_swappable_nodes in seen_referenced_nodes.values():
        if len(seen_swappable_nodes) != 2:
            raise InvalidSwapConfiguration("Swappable nodes should come in pairs!")

    return filtered_launch_description
