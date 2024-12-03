import time
from collections.abc import Callable
from typing import Any, Generic, TypeVar, cast

# Type variable for the method return type
ReturnType = TypeVar("ReturnType")
SomeCallable = Callable[..., ReturnType]


class TTLWrappedFuncType(Generic[ReturnType]):
    """Type hint helper for the decorator"""

    cache: "TTLCache"
    __call__: Callable[..., ReturnType]


class TTLCache:
    def __init__(self, ttl_seconds: float):
        self.ttl_seconds = ttl_seconds
        self._cached_value = None
        self._last_value_timestamp = 0.0
        self._is_set = False  # Indicates whether the cache has been set
        self._get_time = time.time

    def get(self) -> Any:
        if (
            self._is_set
            and self._get_time() - self._last_value_timestamp < self.ttl_seconds
        ):
            return True, self._cached_value
        return False, None

    def set(self, value: Any) -> None:
        self._cached_value = value
        self._last_value_timestamp = self._get_time()
        self._is_set = True


def ttl_cached(
    seconds: float,
) -> Callable[[SomeCallable[ReturnType]], TTLWrappedFuncType[ReturnType]]:
    """
    A function that adds time-to-live (TTL) caching behavior to another function.
    The function's value is cached for a specified number of seconds. After the TTL
    expires, the function is recomputed upon the next access.

    It should be noted that this impelmentation does not (yet) use weakrefs, so the
    cache will hold on to the value until the next recomputation.

    :param seconds: The number of seconds the return value should be cached for.
    :return: A decorator that transforms a class method into a TTL cached function.

    Usage:
    >>> class MyClass:
    ...     @ttl_cached(seconds=10)
    ...     def my_function(self) -> int:
    ...         # Simulate a computation or database access
    ...         return int(time.time())
    ...
    >>> obj = MyClass()
    >>> value1 = obj.my_function()  # This call computes the value
    >>> value2 = obj.my_function()  # This call returns the cached value
    >>> time.sleep(10)
    >>> value3 = obj.my_function()  # This call computes a new value

    The ttl can also be edited, via
    >>> obj.my_function.cache.ttl_seconds = 5
    """

    def decorator(
        func: SomeCallable[ReturnType],
    ) -> TTLWrappedFuncType[ReturnType]:
        cache = TTLCache(seconds)

        def wrapper(*args: Any, **kwargs: Any) -> ReturnType:
            nonlocal cache
            cached, cached_result = cache.get()
            if cached:
                return cast(ReturnType, cached_result)
            else:
                result = func(*args, **kwargs)
                cache.set(result)
                return result

        wrapper = cast(TTLWrappedFuncType[ReturnType], wrapper)

        # Attach the TTLCache instance to the wrapper function for later adjustments
        wrapper.cache = cache
        return wrapper

    return decorator
