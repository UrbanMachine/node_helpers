from typing import cast

import pytest
from launch_ros.actions import Node
from node_helpers.launching import (
    InvalidSwapConfiguration,
    SwapConfiguration,
    SwappableNode,
    apply_node_swaps,
)

namespace_a = "namespace_a"
namespace_b = "namespace_b"
NODE_A = SwappableNode(namespace=namespace_a, name="node_a", executable="")
NODE_A_MOCK = SwappableNode(
    namespace=namespace_a, name="node_a_mock", executable=""
)
NODE_B = SwappableNode(namespace=namespace_b, name="node_b", executable="")
NODE_B_MOCK = SwappableNode(namespace=namespace_b, name="node_b_mock", executable="")
NORMAL_NODE_NAMESPACE_A = Node(namespace=namespace_a, name="cool_node", executable="")


@pytest.mark.parametrize(
    ("config", "input_nodes", "expected_output"),
    (
        # Test a mixed (some mocked some not) example
        (
            {
                namespace_a: SwapConfiguration(
                    mock="node_a_mock", real="node_a", enable_mock=False
                ),
                namespace_b: SwapConfiguration(
                    mock="node_b_mock", real="node_b", enable_mock=True
                ),
            },
            [NODE_A, NODE_A_MOCK, NODE_B, NODE_B_MOCK, NORMAL_NODE_NAMESPACE_A],
            [NODE_A, NODE_B_MOCK, NORMAL_NODE_NAMESPACE_A],
        ),
        # Test no swaps or swap configuration
        ({}, [NORMAL_NODE_NAMESPACE_A], [NORMAL_NODE_NAMESPACE_A]),
        # Test no swap configuration but there are swaps in there
        ({}, [NODE_A, NORMAL_NODE_NAMESPACE_A], InvalidSwapConfiguration),
        # Test when a swap is enabled but its pair node is missing
        (
            {
                namespace_a: SwapConfiguration(
                    mock="node_a_mock", real="node_a", enable_mock=False
                )
            },
            [NODE_A, NORMAL_NODE_NAMESPACE_A],
            InvalidSwapConfiguration,
        ),
        # Test that invalid SwapConfigurations aren't allowed (a configuration where
        # the mock name and real name are the same)
        (
            {
                namespace_a: SwapConfiguration(
                    mock="node_a_mock", real="node_a_mock", enable_mock=False
                )
            },
            [NODE_A, NODE_A_MOCK],
            InvalidSwapConfiguration,
        ),
    ),
)
def test_basic_usage(
    config: dict[str, SwapConfiguration],
    input_nodes: list[SwappableNode | Node],
    expected_output: type[InvalidSwapConfiguration] | list[SwappableNode],
) -> None:
    if expected_output is InvalidSwapConfiguration:
        with pytest.raises(cast(type[Exception], expected_output)):
            apply_node_swaps(configuration=config, launch_description=input_nodes)
    else:
        filtered = apply_node_swaps(
            configuration=config, launch_description=input_nodes
        )
        assert filtered == expected_output
