from warotrans_camera.mjpeg import JpegSplitter


def jpeg(payload: bytes) -> bytes:
    return b"\xff\xd8" + payload + b"\xff\xd9"


def test_splits_concatenated_frames_and_chunk_boundaries():
    a, b = jpeg(b"\x01\xff\x00\x02"), jpeg(b"\x03" * 100)
    stream = b"junk" + a + b
    s = JpegSplitter()
    out = []
    for i in range(0, len(stream), 3):
        out += s.feed(stream[i:i + 3])
    assert out == [a, b]
    assert s.dropped_bytes == 4


def test_marker_split_across_chunks():
    a = jpeg(b"xyz")
    s = JpegSplitter()
    assert s.feed(a[:-1]) == []
    assert s.feed(a[-1:]) == [a]


def test_oversize_garbage_resyncs():
    s = JpegSplitter(max_frame_bytes=50)
    assert s.feed(b"\xff\xd8" + b"\x00" * 100) == []
    a = jpeg(b"ok")
    assert s.feed(a) == [a]
