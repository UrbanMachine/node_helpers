import asyncio
from collections.abc import AsyncGenerator, Coroutine
from contextlib import asynccontextmanager
from typing import Any


@asynccontextmanager
async def run_and_cancel_task(
    coro: Coroutine[Any, Any, Any],
) -> AsyncGenerator[asyncio.Task[Any], None]:
    task = asyncio.create_task(coro)
    try:
        yield task
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
