"""Frame-synchronized video recorder.

Thread-safe write() for use from the camera thread (~30 Hz).
Defers VideoWriter creation until first frame to handle arbitrary resolutions.
"""

import os
import threading
from typing import Tuple, Optional

import cv2


class VideoRecorder:
    """cv2.VideoWriter wrapper. Thread-safe write(). Tracks frame_idx for CSV sync."""

    def __init__(self, path: str, fps: float = 20.0,
                 frame_size: Optional[Tuple[int, int]] = None,
                 fourcc: str = "mp4v"):
        self.path = path
        self.fps = fps
        self._frame_size = frame_size
        self._fourcc_str = fourcc
        self._writer = None
        self._closed = False
        self._lock = threading.Lock()
        self.frame_idx = 0    # monotonically increasing; read from other threads

        # If frame_size provided, open immediately
        if frame_size is not None:
            self._open_writer(frame_size)

    def _open_writer(self, frame_size: Tuple[int, int]) -> None:
        """Open the cv2.VideoWriter with the given frame size."""
        fourcc = cv2.VideoWriter_fourcc(*self._fourcc_str)
        self._writer = cv2.VideoWriter(
            self.path, fourcc, self.fps, frame_size
        )

    def write(self, frame) -> int:
        """Write a BGR frame. Initialize VideoWriter on first frame if deferred.

        Returns the frame_idx that was written (0-based, monotonically increasing).
        Thread-safe.
        """
        with self._lock:
            if self._closed:
                return self.frame_idx
            if self._writer is None:
                # Defer: read frame shape on first write
                h, w = frame.shape[:2]
                self._open_writer((w, h))
                self._frame_size = (w, h)

            if self._writer is not None:
                self._writer.write(frame)
            idx = self.frame_idx
            self.frame_idx += 1
            return idx

    def close(self) -> None:
        """Release VideoWriter. Idempotent."""
        with self._lock:
            self._closed = True
            if self._writer is not None:
                self._writer.release()
                self._writer = None
