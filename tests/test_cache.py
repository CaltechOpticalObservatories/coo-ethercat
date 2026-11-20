"""Tests for the read cache shared by the axes of one backend."""

from __future__ import annotations

import threading
from collections.abc import Callable

import pytest

from cooethercat.cache import ReadCache, read_cache


class Clock:  # pylint: disable=too-few-public-methods
    """A clock the test advances by hand."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def counter() -> tuple[list[int], Callable[[], int]]:
    """Return a call list and a read that appends to it and returns the call count."""
    calls: list[int] = []

    def read() -> int:
        calls.append(1)
        return len(calls)

    return calls, read


def test_value_is_reused_until_max_age() -> None:
    """A value younger than max_age is returned; an older one is read again."""
    clock = Clock()
    cache = ReadCache(clock)
    calls, read = counter()
    assert cache.get("k", read, 0.5) == 1
    clock.now = 0.4
    assert cache.get("k", read, 0.5) == 1
    clock.now = 0.5
    assert cache.get("k", read, 0.5) == 2
    assert len(calls) == 2


def test_max_age_zero_always_reads() -> None:
    """The default max_age of 0 never returns a stored value."""
    cache = ReadCache(Clock())
    calls, read = counter()
    cache.get("k", read)
    cache.get("k", read)
    assert len(calls) == 2


def test_failed_read_is_not_stored() -> None:
    """A read that raises leaves nothing behind, so the next caller reads again."""
    cache = ReadCache(Clock())

    def fail() -> int:
        raise OSError("timeout")

    with pytest.raises(OSError):
        cache.get("k", fail, 10.0)
    assert cache.get("k", lambda: 7, 10.0) == 7


def test_clear_forgets_values() -> None:
    """A value stored before clear is read again after it."""
    cache = ReadCache(Clock())
    calls, read = counter()
    cache.get("k", read, 10.0)
    cache.clear()
    assert cache.get("k", read, 10.0) == 2
    assert len(calls) == 2


def test_read_in_flight_across_clear_is_not_stored() -> None:
    """A value read from before a write is returned to its caller but not kept."""
    cache = ReadCache(Clock())

    def read_then_write() -> int:
        cache.clear()
        return 1

    assert cache.get("k", read_then_write, 10.0) == 1
    assert cache.get("k", lambda: 2, 10.0) == 2


def test_concurrent_readers_of_one_key_share_a_read() -> None:
    """Callers arriving while a read is in flight get its value without reading."""
    cache = ReadCache(Clock())
    started, release = threading.Event(), threading.Event()
    calls: list[int] = []

    def slow() -> int:
        calls.append(1)
        started.set()
        assert release.wait(5.0)
        return 1

    results: list[int] = []
    first = threading.Thread(target=lambda: results.append(cache.get("k", slow, 10.0)))
    first.start()
    assert started.wait(5.0)
    second = threading.Thread(target=lambda: results.append(cache.get("k", slow, 10.0)))
    second.start()
    release.set()
    first.join(5.0)
    second.join(5.0)
    assert results == [1, 1]
    assert len(calls) == 1


def test_slow_key_does_not_hold_up_another() -> None:
    """A read stuck on one key leaves other keys free."""
    cache = ReadCache(Clock())
    started, release = threading.Event(), threading.Event()

    def stuck() -> int:
        started.set()
        release.wait(5.0)
        return 0

    thread = threading.Thread(target=cache.get, args=("slow", stuck, 10.0))
    thread.start()
    try:
        assert started.wait(5.0)
        assert cache.get("fast", lambda: 3, 10.0) == 3
        assert not release.is_set()
    finally:
        release.set()
        thread.join(5.0)


def test_one_cache_per_backend() -> None:
    """Axes built separately on one backend share its cache."""
    first, second = Clock(), Clock()
    assert read_cache(first) is read_cache(first)
    assert read_cache(first) is not read_cache(second)
