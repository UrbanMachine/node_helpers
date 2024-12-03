import random

from std_msgs.msg import ColorRGBA


def color_from_seed(seed: str | float) -> tuple[float, float, float]:
    """Consistently generate a random color from a given seed, as 0-1 floats."""
    rand = random.Random(seed)
    color = (rand.random(), rand.random(), rand.random())
    return color


def color_msg_from_seed(seed: str | float, alpha: float = 1.0) -> ColorRGBA:
    color = color_from_seed(seed)
    return ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
