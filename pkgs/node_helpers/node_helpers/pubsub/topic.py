from collections.abc import Callable
from threading import RLock
from typing import Generic, TypeVar
from weakref import ReferenceType, WeakMethod, ref

from .event import PublishEvent

EMITS = TypeVar("EMITS")
"""The value that gets passed into callbacks"""

SUBSCRIBER = Callable[[EMITS], None]
"""A function that can be called when a topic has a new value"""


class SubscriberNotFoundError(Exception):
    pass


class DuplicateSubscriberError(Exception):
    pass


class Topic(Generic[EMITS]):
    """A topic that can be subscribed to and published to."""

    def __init__(self) -> None:
        self._subscribers: list[ReferenceType[SUBSCRIBER[EMITS]]] = []
        self._subscribers_lock = RLock()

    def subscribe(self, subscriber: SUBSCRIBER[EMITS]) -> None:
        """Adds a new subscriber to the topic.

        Subscribers will be notified of updates in the same thread as the
        publisher, so subscription functions should be thread-safe and should
        avoid doing a lot of work.

        :param subscriber: A function that will be called when the topic is emitted to.

        :raises DuplicateSubscriberError: When subscribing a listener twice to the
            same resource.
        """
        with self._subscribers_lock:
            if self.is_subscribed(subscriber):
                message = "A subscriber already exists for this topic!"
                raise DuplicateSubscriberError(message)

            self._subscribers.append(self._to_weakref(subscriber))

    def subscribe_as_event(self) -> PublishEvent:
        """Adds a new subscriber returns an event object that is set when the topic is
        published to. If the publisher provides any arguments, they will be discarded.

        For example:
        >>> cool_thing_event = topic.subscribe_as_event()
        >>> # Now you can wait blocklingly until the COOL_THING topic is published to
        >>> cool_thing_event.wait_and_clear(timeout=3)

        This is useful in situations where the actual value published doesn't matter,
        but rather, you need to wait until something has been published.

        :return: An event that is set when the topic is published to
        """
        event = PublishEvent()

        with self._subscribers_lock:
            self._subscribers.append(self._to_weakref(event.set_ignore_args))

        return event

    def unsubscribe(self, subscriber: SUBSCRIBER[EMITS]) -> None:
        """Unsubscribe a listener from this topic
        :param subscriber: The callback to unsubscribe from that topic
        """
        with self._subscribers_lock:
            # Find the weakref that refers to this subscriber
            subscriber_weakref = self._to_registered_weakref(subscriber)
            self._subscribers.remove(subscriber_weakref)

    def publish(self, *value: EMITS) -> None:
        """Publishes a new value for a topic.

        :param value: The value to pass on to subscribers
        """
        with self._subscribers_lock:
            for callback_weakref in self._subscribers.copy():
                callback = callback_weakref()
                if callback is None:
                    # This listener has been garbage collected, clean it out
                    self._subscribers.remove(callback_weakref)
                    continue
                callback(*value)

    def is_subscribed(self, subscriber: SUBSCRIBER[EMITS]) -> bool:
        try:
            self._to_registered_weakref(subscriber)
        except SubscriberNotFoundError:
            return False
        else:
            return True

    def _to_weakref(
        self, callback: SUBSCRIBER[EMITS]
    ) -> ReferenceType[SUBSCRIBER[EMITS]]:
        """Convert a callable to a weakref, supporting bound and unbound methods"""
        if hasattr(callback, "__self__") and hasattr(callback, "__func__"):
            return WeakMethod(callback)
        else:
            return ref(callback)

    def _to_registered_weakref(
        self, subscriber: SUBSCRIBER[EMITS]
    ) -> ReferenceType[SUBSCRIBER[EMITS]]:
        """Looks for the weakref for this specifically (already subscribed)
        subscriber. If it's not found, it raises an exception.

        :param subscriber: The callback to store
        :return: The weakref to the subscriber callback
        :raises SubscriberNotFoundError: When a subscriber doesn't exist for this
            particular topic.
        """
        subscriber_weakref = self._to_weakref(subscriber)
        try:
            with self._subscribers_lock:
                weakref = next(s for s in self._subscribers if s == subscriber_weakref)
        except StopIteration as ex:
            raise SubscriberNotFoundError(
                f"Could not find subscriber {subscriber}"
                f"in {self.__class__.__name__}"
            ) from ex
        return weakref


def multi_subscribe_as_event(*topics: Topic[EMITS]) -> PublishEvent:
    """Subscribes to multiple topics and returns an event that is set when any of
    the topics are published to.

    :param topics: The topics to subscribe to
    :return: An event that is set when any of the topics are published to
    """
    event = PublishEvent()

    for topic in topics:
        topic.subscribe(event.set_ignore_args)

    return event
