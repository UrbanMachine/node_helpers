from threading import RLock
from typing import Any


class InvalidRobustMessage(Exception):
    """This exception is raised if a message lacks the required fields for propogating
    error information.
    """


class ActionCancellationRejected(Exception):
    """Raised if it was implied an action needed to be cancelled, but the cancellation
    request was rejected.
    """


class ExecutorNotSetError(Exception):
    """Raised if a service or action attempts to send a request before the nodes
    executor has been set. This can happen when running an action in the __init__ of a
    node."""


_ROBUST_RPC_EXCEPTION_CACHE_LOCK = RLock()
_ROBUST_RPC_EXCEPTION_CACHE: dict[str, type["RobustRPCException"]] = {}
"""This dict carries a mapping of 'error_name' to a dynamically generated exception
object of the same name. """


class RobustRPCException(Exception):
    def __init__(self, error_name: str, error_description: str, message: Any):
        super().__init__(
            f"RPC Call failed with exception {error_name}('{error_description}')"
        )
        self.error_name = error_name
        self.error_description = error_description
        self.message = message

    @classmethod
    def like(
        cls: type["RobustRPCException"], exception: str | type[Exception]
    ) -> "type[RobustRPCException]":
        """The most ergonomic way to catch errors that occured remotely!

        This function returns an error 'like' the one passed in, but that subclasses
        a RobustRPCException, and will match the same error object that would be raised
        for an error of the same name.

        :param exception: The object or name you want to create a mirroring error object
            of.
        :return: An object with the same name as 'exception', but that subclasses
            RobustRPCException.

        Usage example:
        >>> try:
        >>>     maybe_raises_an_rpc_error()
        >>> except RobustRPCException.like(OutOfBoundsError):
        >>>     # In this case, the 'like' function will take the .__class__.__name__
        >>>     # of the error to create a new one that subclasses RobustRPCException
        >>>     handle_out_of_bounds_case()
        >>> except RobustRPCException.like("ErrorFromAPackageThatsNotADependency"):
        >>>     # In this case, the string is used as the name of the new error object
        >>>     handle_error()
        >>> except RobustRPCException:
        >>>     # All errors still subclass RobustRPCException and can be caught
        >>>     handle_all_other_robust_rpc_errors()

        Some useful properties to explain the output error object:
        >>> assert RobustRPCException.like(OutOfBoundsError) != OutOfBoundsError
        >>> assert RobustRPCException.like(OutOfBoundsError) is not OutOfBoundsError
        >>> assert (
        >>>     RobustRPCException.like(OutOfBoundsError).__name__
        >>>     == OutOfBoundsError.__name__
        >>> )
        """
        ex_name: str = exception if isinstance(exception, str) else exception.__name__

        with _ROBUST_RPC_EXCEPTION_CACHE_LOCK:
            if ex_name not in _ROBUST_RPC_EXCEPTION_CACHE:
                _ROBUST_RPC_EXCEPTION_CACHE[ex_name] = type(ex_name, (cls,), {})

            return _ROBUST_RPC_EXCEPTION_CACHE[ex_name]
