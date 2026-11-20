"""Read cache shared by every Axis and AxisSet on one backend.

A caller asks for a value no older than ``max_age`` seconds; 0, the default
everywhere, always reads. Readers of one key share a single read, readers of
different keys never wait on each other, and a failed read is not stored. Any
write through the axis API clears the cache, so a read after a write is fresh.
"""

from __future__ import annotations

import threading
import time
import weakref
from collections.abc import Callable, Hashable
from typing import TypeVar

T = TypeVar("T")


class ReadCache:
    """Values keyed by what was read, each stamped with when its read began."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._entries: dict[Hashable, tuple[float, object]] = {}
        self._key_locks: dict[Hashable, threading.Lock] = {}
        # Bumped by clear, so a read in flight across a write is not stored
        self._generation = 0

    def get(self, key: Hashable, read: Callable[[], T], max_age: float = 0.0) -> T:
        """Return the value under key if younger than max_age seconds, else read it."""
        with self._lock:
            key_lock = self._key_locks.setdefault(key, threading.Lock())
        with key_lock:
            if max_age > 0:
                with self._lock:
                    entry = self._entries.get(key)
                if entry is not None and self._clock() - entry[0] < max_age:
                    return entry[1]  # type: ignore[return-value]
            with self._lock:
                generation, started = self._generation, self._clock()
            value = read()
            with self._lock:
                if generation == self._generation:
                    self._entries[key] = (started, value)
            return value

    def clear(self) -> None:
        """Forget every value, including any read still in flight."""
        with self._lock:
            self._entries.clear()
            self._generation += 1


_CACHES: weakref.WeakKeyDictionary[object, ReadCache] = weakref.WeakKeyDictionary()
_CACHES_LOCK = threading.Lock()


def read_cache(backend: object) -> ReadCache:
    """Return the cache of one backend, so axes built separately on it share it."""
    with _CACHES_LOCK:
        cache = _CACHES.get(backend)
        if cache is None:
            cache = _CACHES[backend] = ReadCache()
        return cache
