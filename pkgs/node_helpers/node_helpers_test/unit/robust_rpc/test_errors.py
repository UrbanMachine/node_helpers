import pytest
from node_helpers.robust_rpc import RobustRPCException


class CoolError(Exception):
    pass


def test_caching() -> None:
    assert RobustRPCException.like("CoolError") is RobustRPCException.like(CoolError)
    assert CoolError is not RobustRPCException.like(CoolError)  # type: ignore
    assert RobustRPCException.like("CoolError").__name__ == CoolError.__name__


def test_routing_exceptions_works() -> None:
    """This test is a sanity check of how RobustRPCExceptions are supposed to work"""
    try:
        raise RobustRPCException.like("CoolError")(
            error_name="CoolError", error_description="", message=None
        )
    except RobustRPCException.like("NotCoolError"):
        raise
    except RobustRPCException.like(CoolError):
        routed_correctly = True
    except RobustRPCException:
        raise

    assert routed_correctly


def test_routing_accepts_robust_rpc_exception() -> None:
    """Test that the 'like' function returns an object that subclasses
    RobustRPCException
    """
    try:
        raise RobustRPCException.like("CoolError")(
            error_name="CoolError", error_description="", message=None
        )
    except RobustRPCException.like("NotCoolError"):
        raise
    except RobustRPCException:
        routed_correctly = True

    assert routed_correctly


def test_routing_falls_through() -> None:
    """A simple sanity check that one exception doesn't get matched with another of a
    different name"""

    with pytest.raises(RobustRPCException.like("CoolError")):
        try:
            raise RobustRPCException.like("CoolError")(
                error_name="CoolError", error_description="", message=None
            )
        except RobustRPCException.like("NotCoolError"):
            pass
