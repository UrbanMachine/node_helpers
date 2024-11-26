import gc
from queue import Queue

import pytest
from node_helpers.pubsub import DuplicateSubscriberError, SubscriberNotFoundError, Topic


def test_publishing_subscribing() -> None:
    """A basic test of Publisher/Subscriber functionality"""
    topic = Topic[str]()
    queue_1: Queue[str] = Queue()

    # Smoke test
    topic.publish("no one should get this!")
    assert queue_1.qsize() == 0
    topic.subscribe(queue_1.put)
    topic.publish("topic_1_content")
    assert queue_1.qsize() == 1
    assert queue_1.get() == "topic_1_content"


def test_gc_listener_gets_removed() -> None:
    """A test to make sure that if a listener is garbage collected, it is
    removed from the Queue after the next time publish is called"""
    topic = Topic[str]()
    queue_1: Queue[str] = Queue()

    # Subscribe the listener, queue_1.put, and verify there is a weak ref
    assert len(topic._subscribers) == 0
    topic.subscribe(queue_1.put)
    assert len(topic._subscribers) == 1

    # Publish some content
    topic.publish("alex is cool")
    assert queue_1.get() == "alex is cool"

    # Delete the listener and garbage collect
    del queue_1
    gc.collect()

    # Test the listener is deleted the next time something is published
    assert len(topic._subscribers) == 1
    topic.publish("i am a string look at me")
    assert len(topic._subscribers) == 0


def test_removing_listener() -> None:
    """Test removing subscribers from a topic works"""
    topic = Topic[str]()
    queue_1: Queue[str] = Queue()
    queue_2: Queue[str] = Queue()

    # Send a method to two channels
    topic.subscribe(queue_1.put)
    topic.subscribe(queue_2.put)
    topic.publish("to_both")
    assert queue_1.get() == "to_both"
    assert queue_2.get() == "to_both"

    # Remove a listener, and make sure it gets nothing
    topic.unsubscribe(queue_1.put)
    topic.publish("to_queue_2")
    assert queue_2.get() == "to_queue_2"
    assert queue_1.qsize() == 0

    # Test removing an already removed listener raises an error
    with pytest.raises(SubscriberNotFoundError):
        topic.unsubscribe(queue_1.put)


def test_no_duplicate_subscribers() -> None:
    topic = Topic[None]()
    queue: Queue[None] = Queue()

    topic.subscribe(queue.put)

    # This should not be okay
    with pytest.raises(DuplicateSubscriberError):
        topic.subscribe(queue.put)

    # A second (new) subscriber should be okay
    second_queue: Queue[None] = Queue()
    topic.subscribe(second_queue.put)


def test_subscribe_call_counts() -> None:
    """Tests that subscribers are notified when a topic's value changes"""
    topic = Topic[int]()

    subscriber1_call_count = 0

    def subscriber1(value: int) -> None:
        assert value == 5
        nonlocal subscriber1_call_count
        subscriber1_call_count += 1

    topic.subscribe(subscriber1)

    subscriber2_call_count = 0

    def subscriber2(value: int) -> None:
        assert value == 5
        nonlocal subscriber2_call_count
        subscriber2_call_count += 1

    topic.subscribe(subscriber2)

    topic.publish(5)

    assert subscriber1_call_count == 1
    assert subscriber2_call_count == 1


def test_publish_no_subscribers() -> None:
    """Tests that no errors occur when publishing to a topic with no
    subscribers.
    """
    topic = Topic[int]()
    topic.publish(8)


def test_subscribe_as_event() -> None:
    """Tests that event subscribers are notified of a publish"""
    topic = Topic[str]()

    event = topic.subscribe_as_event()
    assert not event.is_set(), "The event was set before a publish"

    # Tests that the event subscriber receives the publish
    topic.publish("Hello event!", "You should be set")
    assert event.is_set(), "The event was not set after a publish"
    event.wait_and_clear()

    assert not event.is_set()


def test_gc_subscribe_as_event_removed() -> None:
    """Tests that the event subscription is removed when the event is garbage
    collected.
    """
    topic = Topic[str]()

    event = topic.subscribe_as_event()
    assert len(topic._subscribers) == 1

    del event
    gc.collect()

    assert len(topic._subscribers) == 1
    topic.publish("What is good")
    assert len(topic._subscribers) == 0


def test_multiple_subscribe_as_event() -> None:
    """Tests that all subscribed events are notified with a single publish"""
    topic = Topic[str]()

    event1 = topic.subscribe_as_event()
    event2 = topic.subscribe_as_event()

    topic.publish("What's up, my events!")

    assert event1.is_set()
    assert event2.is_set()
