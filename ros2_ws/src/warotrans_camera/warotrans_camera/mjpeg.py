"""Split a raw MJPEG byte stream (concatenated JPEG images, e.g. `rpicam-vid --codec mjpeg -o -`) into frames.

JPEG entropy data byte-stuffs 0xFF (0xFF00), so FFD8 (SOI) and FFD9 (EOI) only appear as markers. rpicam-vid
MJPEG frames carry no embedded thumbnail, so the first EOI after an SOI ends the frame.
"""

from __future__ import annotations

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


class JpegSplitter:
    def __init__(self, max_frame_bytes: int = 4 * 1024 * 1024) -> None:
        self._buf = bytearray()
        self._max = max_frame_bytes
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> list[bytes]:
        self._buf.extend(data)
        frames: list[bytes] = []
        while True:
            start = self._buf.find(SOI)
            if start < 0:
                keep = 1 if self._buf[-1:] == b"\xff" else 0
                self.dropped_bytes += len(self._buf) - keep
                del self._buf[: len(self._buf) - keep]
                return frames
            if start:
                self.dropped_bytes += start
                del self._buf[:start]
            end = self._buf.find(EOI, 2)
            if end < 0:
                if len(self._buf) > self._max:  # corrupt stream: resync on the next SOI
                    self.dropped_bytes += len(self._buf)
                    self._buf.clear()
                return frames
            frames.append(bytes(self._buf[: end + 2]))
            del self._buf[: end + 2]
