import logging
import traceback
from collections.abc import Callable
from typing import Any

from rclpy import Future
from rclpy.action.server import ServerGoalHandle

from .errors import RobustRPCException
from .schema import ERROR_DESCRIPTION_FIELD, ERROR_NAME_FIELD
from .typing import RequestType, ResponseType

ServiceCallback = Callable[[RequestType, ResponseType], ResponseType]
ActionCallback = Callable[[ServerGoalHandle], ResponseType]


def patch_future_to_raise_exception(future: Future, parse_as_action: bool) -> None:
    """Patches the future.result() method to raise an error if the result from the
    service or action has an error set within the resulting message.
    :param future: The future to patch the result() method of
    :param parse_as_action: If True, it will expect the exception information to be
        nested within the "result" attribute, as actions do. Otherwise, it will expect
        the exception name and description to be root object attributes.
    """

    def result_patch() -> Any:
        if future._exception:  # noqa: SLF001
            # If the future failed for other reasons, prioritize that exception
            raise future.exception()

        result = future._result  # noqa: SLF001
        msg_with_error_info = result.result if parse_as_action else result
        msg_error = getattr(msg_with_error_info, ERROR_NAME_FIELD)

        if msg_error != "":
            # Since an error occurred, raise the appropriate error type
            msg_error_description = getattr(
                msg_with_error_info, ERROR_DESCRIPTION_FIELD
            )
            error_class = RobustRPCException.like(msg_error)
            raise error_class(
                error_name=msg_error,
                error_description=msg_error_description,
                message=result,
            )

        return result

    future.result = result_patch


def wrap_service_callback(callback: ServiceCallback, srv_name: str) -> ServiceCallback:
    """Wraps a service callback, catches errors, and puts them in service message"""

    def wrapper(request: RequestType, response: ResponseType) -> ResponseType:
        try:
            return callback(request, response)
        except Exception as ex:  # noqa: BLE001
            _add_error_info(ex, srv_name, response)
            return response

    return wrapper


def wrap_action_callback(
    callback: ActionCallback, action_name: str, result_type: type[ResponseType]
) -> ActionCallback:
    """Wraps an action callback, catches errors, and puts them in the action message."""

    def wrapper(goal: ServerGoalHandle) -> ResponseType:
        try:
            return callback(goal)
        except Exception as ex:  # noqa: BLE001
            goal.abort()
            response = result_type()
            _add_error_info(ex, action_name, response)
            return response

    return wrapper


def _add_error_info(ex: Exception, rpc_name: str, result: ResponseType) -> None:
    """Adds error information to the provided result if the result type has the
    necessary fields

    :param ex: The exception to get information on
    :param rpc_name: The name of the service or action that has failed
    :param result: The result to provide the information to
    """
    if isinstance(ex, RobustRPCException):
        # Pass through the pre-existing name and description
        name = ex.error_name
        description = ex.error_description
    else:
        name = type(ex).__name__
        description = str(ex)

    logging.error(
        f"Exception while running RPC '{rpc_name}':\n{traceback.format_exc()}"
    )
    setattr(result, ERROR_NAME_FIELD, name)
    setattr(result, ERROR_DESCRIPTION_FIELD, description)


__all__ = [
    "wrap_action_callback",
    "wrap_service_callback",
    "patch_future_to_raise_exception",
]
