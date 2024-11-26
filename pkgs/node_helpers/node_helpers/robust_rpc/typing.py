from typing import Protocol


class RequestType(Protocol):
    """For type hinting a service_msg.Request or an action_msg.Goal"""


class ResponseType(Protocol):
    """For type hinting a service_msg.Response or an action_msg.Result"""

    error_name: str
    error_description: str


class RobustServiceMsg(Protocol):
    Request: type[RequestType]
    Response: type[ResponseType]


class RobustActionMsg(Protocol):
    Goal: type[RequestType]
    Result: type[ResponseType]
