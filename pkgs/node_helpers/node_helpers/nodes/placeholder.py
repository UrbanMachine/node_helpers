from typing import Any

from node_helpers.nodes import HelpfulNode
from node_helpers.spinning import create_spin_function


class Placeholder(HelpfulNode):
    """A placeholder node that can be used in launch files to keep them running even
    if no other nodes are present
    """

    def __init__(self, **kwargs: Any):
        super().__init__("placeholder", **kwargs)


main = create_spin_function(Placeholder)
