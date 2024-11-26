from abc import ABC, abstractmethod
from typing import cast

from node_helpers_msgs.msg import UserPrompt
from pydantic import BaseModel


class PromptMetadata(ABC, BaseModel):
    """Defines the metadata field's schema for a UserPrompt type. See the
    message definition for UserPrompt for more information.
    """

    @staticmethod
    @abstractmethod
    def type() -> int:
        """The prompt type that this metadata is used with"""


class BasicPromptMetadata(PromptMetadata):
    @staticmethod
    def type() -> int:
        return cast(int, UserPrompt.PROMPT_BASIC)


class TeleopPromptMetadata(PromptMetadata):
    namespaces: list[str]

    @staticmethod
    def type() -> int:
        return cast(int, UserPrompt.PROMPT_TELEOP)
