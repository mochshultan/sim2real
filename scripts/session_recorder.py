"""Bounded, asynchronous JSONL recording; no disk writes in the control loop."""

from datetime import datetime, timezone
import json
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import time
from uuid import uuid4


class SessionRecorder:
    def __init__(self, directory, metadata, queue_size=512):
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.path = directory / f'sim2real_zero_{stamp}_{uuid4().hex[:8]}.jsonl'
        self.dropped = 0
        self.error = None
        self._closed = False
        self._queue = Queue(maxsize=queue_size)
        self._stream = self.path.open('x', encoding='utf-8')
        try:
            self._stream.write(json.dumps({'type': 'metadata', 'schema_version': 1, **metadata}) + '\n')
            self._stream.flush()
        except Exception:
            self._stream.close()
            raise
        self._thread = threading.Thread(target=self._write, name='session-recorder', daemon=True)
        self._thread.start()

    def record(self, sample):
        """Enqueue an owned snapshot; never wait for the writer or storage."""
        if self._closed or self.error is not None:
            self.dropped += 1
            return
        try:
            self._queue.put_nowait(sample)
        except Full:
            self.dropped += 1

    def _write(self):
        try:
            last_flush = time.monotonic()
            while True:
                try:
                    sample = self._queue.get(timeout=0.5)
                except Empty:
                    self._stream.flush()
                    last_flush = time.monotonic()
                    continue
                if sample is None:
                    break
                self._stream.write(json.dumps(sample, separators=(',', ':')) + '\n')
                if time.monotonic() - last_flush >= 1.0:
                    self._stream.flush()
                    last_flush = time.monotonic()
            self._stream.write(json.dumps({'type': 'summary', 'dropped_samples': self.dropped}) + '\n')
        except Exception as exc:
            self.error = str(exc)
        finally:
            try:
                self._stream.close()
            except Exception as exc:
                self.error = str(exc)

    def close(self):
        if not self._closed:
            self._closed = True
            deadline = time.monotonic() + 5.0
            while self._thread.is_alive():
                try:
                    self._queue.put(None, timeout=0.1)
                    break
                except Full:
                    if time.monotonic() >= deadline:
                        break
            self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        return not self._thread.is_alive() and self.error is None
