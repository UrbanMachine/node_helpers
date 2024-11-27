from rclpy.executors import MultiThreadedExecutor


class MultiThreadedStackTracedExecutor(MultiThreadedExecutor):
    """This modifies the MultiThreadedExecutor to ensure that the 'shutdown()' call
    blocks untill all underlying threads have joined.
    """

    def shutdown(
        self, timeout_sec: float | None = None, wait_for_threads: bool = True
    ) -> bool:
        """
        Stop executing callbacks and wait for their completion.

        :param timeout_sec: Seconds to wait. Block forever if ``None`` or negative.
            Don't wait if 0.
        :param wait_for_threads: If true, this function will exit only once all executor
            threads have joined.
        :return: ``True`` if all outstanding callbacks finished executing, or ``False``
            if the timeout expires before all outstanding work is done.
        """
        success: bool = super().shutdown(timeout_sec)
        self._executor.shutdown(wait=wait_for_threads)
        return success


__all__ = ["MultiThreadedStackTracedExecutor"]
