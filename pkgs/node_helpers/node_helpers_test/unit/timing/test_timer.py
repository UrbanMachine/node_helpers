import pytest
from node_helpers.timing import Timer


def test_context_manager() -> None:
    """Test the context manager of the timer utility works as expected"""
    n_samples = 5
    timer = Timer(samples=n_samples)

    for i in range(n_samples):
        with timer:
            pass
        assert len(timer._samples) == i + 1

    with timer:
        pass

    # Verify the timer doesn't keep samples over the specified amount
    assert len(timer._samples) == n_samples
    assert timer.fps > 0
    assert timer.elapsed > 0


def test_decorator() -> None:
    """Test the decorator functionality of the timer utility works as expected"""
    timer = Timer(samples=3)

    @timer
    def my_cool_func() -> int:
        return 3

    retval = my_cool_func()
    assert retval == 3
    assert len(timer._samples) == 1

    my_cool_func()
    assert len(timer._samples) == 2


def test_printing_doesnt_cause_zero_division_error() -> None:
    timer = Timer(samples=100)
    assert len(timer._samples) == 0
    assert repr(timer) == f"Timer({Timer._NO_SAMPLES_MSG})"

    with timer:
        pass

    assert len(timer._samples) == 1
    assert repr(timer) != f"Timer({Timer._NO_SAMPLES_MSG})"


def test_child_gets_parameters_passed() -> None:
    timer = Timer(samples=32)
    child_name = "cool-functionality"
    assert len(timer._children) == 0

    # Create the child
    with timer:
        timer.child(child_name)

    assert len(timer._children) == 1
    assert timer._children[child_name].name == child_name
    assert timer._children[child_name]._num_samples == timer._num_samples

    # Add samples to the child
    with timer, timer.child(child_name):
        pass

    assert len(timer._children[child_name]._samples) == 1


def test_report_generation() -> None:
    name = "cool_name"
    timer = Timer(samples=42, name=name)
    assert name.title() in str(timer)
    assert Timer._NO_SAMPLES_MSG in str(timer)
    assert Timer._REPORT_INDENT not in str(timer)

    with timer:
        pass

    assert name.title() in str(timer)
    assert Timer._REPORT_INDENT not in str(timer)

    # Test children
    with timer, timer.child("thing"):
        pass
    assert str(timer).count(Timer._REPORT_INDENT) == 1


def test_enter_child_outside_of_context() -> None:
    timer = Timer()
    timer_name = "cool-child"

    with pytest.raises(RuntimeError):
        timer.child(timer_name)

    assert len(timer._children) == 0


def test_zero_samples_behavior() -> None:
    """The timer shouldn't throw errors when 'elapsed' and 'fps' are called"""
    timer = Timer()

    assert timer.elapsed == 0.0
    assert timer.total_elapsed == 0.0
    assert timer.fps == 0.0
    assert isinstance(timer.elapsed, float)
    assert isinstance(timer.total_elapsed, float)
    assert isinstance(timer.fps, float)
