from threading import Event
from typing import Any


class PublishEvent(Event):
    """An event with a method that can throw away arguments given to it by a
    publisher, and therefore can be used as a subscriber in
    ``Topic.subscribe_as_event``.

    Why use this instead of a simple function or lambda that throws away
    arguments and calls ``event.set()`` instead? Using this method ensures that
    the subscription is garbage collected once this event object falls out of
    scope. If we used a weakref to a function or lambda instead, the function
    would fall out of scope immediately after calling
    ``Topic.subscribe_as_event`` and get garbage collected. If it was a
    regular reference to a function or lambda, it would never fall out of scope
    and callers would need to remember to unsubscribe the event.
    """

    def set_ignore_args(self, *_args: Any, **_kwargs: Any) -> None:
        self.set()

    def wait_and_clear(self, timeout: float | None = None) -> bool:
        """This helps reinforce a pattern of 'waiting' then 'clearing' an event.

        Here's an example of waiting on an event with a bug:
        >>> while a_value_changed_event.wait():
        >>>     if a_value:
        >>>         break

        In this example, the first time the value changes it will set the event to True,
        and now you have a busy loop on your hands. Here's the fixed version:
        >>> while a_value_changed_event.wait_and_clear():
        >>>     if a_value:
        >>>         break

        Now, it will wait on the event, clear the event condition, and check the value.
        Next time it hits the 'while' line, it will wait again!

        :param timeout: The timeout in seconds for the event.wait() call
        :return: True, if the condition was set
        """
        was_set = self.wait(timeout=timeout)
        if was_set:
            # Only run clear if the event was set, avoiding a race condition where
            # was_set is False, but it becomes set and is immediately cleared after.
            self.clear()
        return was_set
