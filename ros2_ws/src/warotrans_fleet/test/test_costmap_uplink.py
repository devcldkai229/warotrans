"""Live costmap uplink: compact, keeps the most restrictive cost, unknown stays unknown."""

import base64

from warotrans_fleet.costmap_uplink import UNKNOWN, build_message, encode_grid


def test_small_grid_is_sent_as_is_with_unknown_mapped() -> None:
    data, w, h, f = encode_grid([0, 50, -1, 100], 2, 2)
    assert (w, h, f) == (2, 2, 1)
    assert list(data) == [0, 50, UNKNOWN, 100]


def test_large_grid_is_max_pooled_so_obstacles_survive() -> None:
    width = height = 320
    values = [0] * (width * height)
    values[5 * width + 7] = 100                      # one lethal cell
    data, w, h, f = encode_grid(values, width, height, max_cells=160)
    assert (w, h, f) == (160, 160, 2)
    assert data[2 * 160 + 3] == 100                  # it is still lethal after pooling


def test_message_carries_geometry_for_the_map() -> None:
    m = build_message("RBT-001", "local", "map", 0.05, -1.0, -2.0, [0, 99], 2, 1, 123.0)
    assert m["kind"] == "local" and m["rowOrder"] == "bottom-up"
    assert (m["width"], m["height"], m["resolution"], m["originX"], m["originY"]) == (2, 1, 0.05, -1.0, -2.0)
    assert list(base64.b64decode(m["data"])) == [0, 99]
