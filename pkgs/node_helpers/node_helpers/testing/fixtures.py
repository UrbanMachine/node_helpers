from collections.abc import Generator

import pytest

from .threads import get_unclosed_threads


@pytest.fixture(autouse=True)
def each_test_setup_teardown() -> Generator[None, None, None]:  # noqa: PT004
    """This function should be imported in the root `conftest.py` of every package.

    It will validate that all threads have been closed inbetween each test, and end the
    testing session if not.

    :yields: Nothing. There's no need to depend on this fixture as long as it's imported
    """
    yield

    # Any teardown for all tests goes here
    unclosed_threads = get_unclosed_threads()
    if len(unclosed_threads):
        msg = (
            "There were unclosed threads after fixture teardown: "
            f"{unclosed_threads=}.\n The testing session will end unfinished."
        )

        pytest.exit(msg, returncode=-1)
