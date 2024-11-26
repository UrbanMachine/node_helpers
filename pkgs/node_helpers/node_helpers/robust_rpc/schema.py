from .errors import InvalidRobustMessage
from .typing import ResponseType

ERROR_NAME_FIELD = "error_name"
ERROR_DESCRIPTION_FIELD = "error_description"


def validate_robust_message(message: type[ResponseType]) -> None:
    """Validates that a message can be used in the robust RPC framework

    This necessitates having the error name and error description fields.

    :param message: Either a Service or Action type message to check the fields for
    :raises InvalidRobustMessage: If any required fields are missing.
    """
    has_name_field = hasattr(message, ERROR_NAME_FIELD)
    has_description_field = hasattr(message, ERROR_DESCRIPTION_FIELD)
    if not has_name_field or not has_description_field:
        raise InvalidRobustMessage(
            f"The message {message} is missing required fields: "
            f"{ERROR_NAME_FIELD}, {ERROR_DESCRIPTION_FIELD}"
        )
