"""Minimal AVI 1.0 writer for MJPEG frames (no ffmpeg on the Pi, no re-encoding).

The frames rpicam-vid already compressed are stored as they are ('00dc' chunks) with an idx1 index, which is what
AWS Elemental MediaConvert accepts as "MJPEG in AVI" (it does not accept a raw .mjpeg stream). The frame rate is
constant: the caller (RecorderSink) fills fixed time slots, repeating or skipping frames, so media time follows the
capture clock.

Layout:
    RIFF 'AVI '
      LIST 'hdrl'  avih, LIST 'strl' (strh 'vids' 'MJPG', strf BITMAPINFOHEADER)
      LIST 'movi'  '00dc' <jpeg> ...
      idx1         one 16-byte entry per frame (offsets from the 'movi' fourcc)

AVI 1.0 is limited to 1 GiB: a segment of 30 s at 1280x720 MJPEG is about 60 MB, far below.
"""

from __future__ import annotations

import os
import struct
from typing import BinaryIO, Optional

AVIF_HASINDEX = 0x10
AVIIF_KEYFRAME = 0x10
MAX_BYTES = 1 << 30


class AviMjpegWriter:
    def __init__(self, path: str, width: int, height: int, fps: int) -> None:
        if width <= 0 or height <= 0 or not 0 < fps <= 120:
            raise ValueError("width, height and fps must be positive (fps <= 120)")
        self.path = path
        self.width, self.height, self.fps = width, height, fps
        self._f: Optional[BinaryIO] = open(path, "wb")
        self._index: list[tuple[int, int]] = []  # (offset from 'movi' fourcc, size)
        self._max_frame = 0
        self._write_headers()
        self._movi_fourcc_pos = self._movi_list_pos + 8

    # ------------------------------------------------------------------ public

    @property
    def frame_count(self) -> int:
        return len(self._index)

    @property
    def bytes_written(self) -> int:
        return self._f.tell() if self._f else os.path.getsize(self.path)

    def write_frame(self, jpeg: bytes) -> None:
        if self._f is None:
            raise ValueError("writer is closed")
        if not jpeg.startswith(b"\xff\xd8"):
            raise ValueError("not a JPEG frame")
        if self._f.tell() + len(jpeg) + 64 + 16 * (len(self._index) + 1) > MAX_BYTES:
            raise OverflowError("AVI 1.0 segment would exceed 1 GiB")
        offset = self._f.tell() - self._movi_fourcc_pos
        self._f.write(b"00dc" + struct.pack("<I", len(jpeg)) + jpeg)
        if len(jpeg) % 2:
            self._f.write(b"\x00")  # chunks are word aligned
        self._index.append((offset, len(jpeg)))
        self._max_frame = max(self._max_frame, len(jpeg))

    def close(self, fsync: bool = True) -> None:
        """Writes the index, patches sizes and counts, flushes (and fsyncs) the file."""
        if self._f is None:
            return
        f = self._f
        movi_end = f.tell()
        f.write(b"idx1" + struct.pack("<I", 16 * len(self._index)))
        for offset, size in self._index:
            f.write(b"00dc" + struct.pack("<III", AVIIF_KEYFRAME, offset, size))
        end = f.tell()
        # sizes
        self._patch(4, end - 8)                                   # RIFF size
        self._patch(self._movi_list_pos + 4, movi_end - self._movi_list_pos - 8)
        # counts
        frames = len(self._index)
        self._patch(self._avih_pos + 8 + 16, frames)              # avih.dwTotalFrames
        self._patch(self._avih_pos + 8 + 28, self._max_frame)     # avih.dwSuggestedBufferSize
        self._patch(self._strh_pos + 8 + 32, frames)              # strh.dwLength
        self._patch(self._strh_pos + 8 + 36, self._max_frame)     # strh.dwSuggestedBufferSize
        f.flush()
        if fsync:
            os.fsync(f.fileno())
        f.close()
        self._f = None

    def abort(self) -> None:
        if self._f is not None:
            self._f.close()
            self._f = None

    # ------------------------------------------------------------------ layout

    def _write_headers(self) -> None:
        f = self._f
        assert f is not None
        f.write(b"RIFF" + struct.pack("<I", 0) + b"AVI ")
        hdrl_pos = f.tell()
        f.write(b"LIST" + struct.pack("<I", 0) + b"hdrl")
        self._avih_pos = f.tell()
        f.write(b"avih" + struct.pack("<I", 56))
        f.write(struct.pack(
            "<IIIIIIIIII16x",
            round(1_000_000 / self.fps),   # dwMicroSecPerFrame
            0,                             # dwMaxBytesPerSec
            0,                             # dwPaddingGranularity
            AVIF_HASINDEX,                 # dwFlags
            0,                             # dwTotalFrames (patched)
            0,                             # dwInitialFrames
            1,                             # dwStreams
            0,                             # dwSuggestedBufferSize (patched)
            self.width, self.height))
        strl_pos = f.tell()
        f.write(b"LIST" + struct.pack("<I", 0) + b"strl")
        self._strh_pos = f.tell()
        f.write(b"strh" + struct.pack("<I", 56))
        f.write(b"vids" + b"MJPG")
        f.write(struct.pack(
            "<IHHIIIIIIIIhhhh",
            0,             # dwFlags
            0, 0,          # wPriority, wLanguage
            0,             # dwInitialFrames
            1,             # dwScale
            self.fps,      # dwRate  -> fps = rate / scale
            0,             # dwStart
            0,             # dwLength (patched)
            0,             # dwSuggestedBufferSize (patched)
            0xFFFFFFFF,    # dwQuality (-1 = default)
            0,             # dwSampleSize (0 = variable size frames)
            0, 0, self.width, self.height))
        f.write(b"strf" + struct.pack("<I", 40))
        f.write(struct.pack("<IiiHH4sIiiII", 40, self.width, self.height, 1, 24, b"MJPG",
                            self.width * self.height * 3, 0, 0, 0, 0))
        strl_end = f.tell()
        self._patch(strl_pos + 4, strl_end - strl_pos - 8)
        self._patch(hdrl_pos + 4, strl_end - hdrl_pos - 8)
        self._movi_list_pos = f.tell()
        f.write(b"LIST" + struct.pack("<I", 0) + b"movi")

    def _patch(self, pos: int, value: int) -> None:
        f = self._f
        assert f is not None
        here = f.tell()
        f.seek(pos)
        f.write(struct.pack("<I", value))
        f.seek(here)


def read_avi_index(path: str) -> dict:
    """Reads back what a player needs (tests and the readiness check): frame rate, size, count and every frame."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"RIFF" or data[8:12] != b"AVI ":
        raise ValueError("not an AVI file")
    if struct.unpack_from("<I", data, 4)[0] != len(data) - 8:
        raise ValueError("RIFF size does not match the file")
    avih = data.index(b"avih")
    micro, _, _, flags, total = struct.unpack_from("<IIIII", data, avih + 8)
    width, height = struct.unpack_from("<II", data, avih + 8 + 32)
    strh = data.index(b"strh")
    scale, rate = struct.unpack_from("<II", data, strh + 8 + 20)
    length = struct.unpack_from("<I", data, strh + 8 + 32)[0]
    movi = data.index(b"movi")
    idx1 = data.index(b"idx1", movi)
    count = struct.unpack_from("<I", data, idx1 + 4)[0] // 16
    frames = []
    for i in range(count):
        ckid, flag, offset, size = struct.unpack_from("<4sIII", data, idx1 + 8 + 16 * i)
        pos = movi + offset
        if data[pos:pos + 4] != ckid or struct.unpack_from("<I", data, pos + 4)[0] != size:
            raise ValueError(f"index entry {i} does not point at its chunk")
        frames.append(data[pos + 8:pos + 8 + size])
    return {"fps": rate / scale, "width": width, "height": height, "total_frames": total, "length": length,
            "has_index": bool(flags & AVIF_HASINDEX), "frames": frames, "micro_sec_per_frame": micro}
