import math

import pytest

from warotrans_hardware.serial_protocol import (
    format_config_command,
    format_velocity_command,
    parse_battery_line,
    parse_config_ack,
    parse_encoder_line,
    wheel_geometry_commands,
)


def test_parse_valid_encoder_line_bytes():
    assert parse_encoder_line(b"O 120 -85 5020\n") == (120, -85, 5020)


def test_parse_valid_encoder_line_with_extra_whitespace():
    assert parse_encoder_line("  O 1 2 3  \r\n") == (1, 2, 3)


def test_rejects_malformed_encoder_lines():
    malformed = (
        b"",
        b"V 1 2\n",
        b"O 1 2\n",
        b"O 1 2 3 4\n",
        b"O one 2 3\n",
        b"O 1 2 -1\n",
        b"O 1 2 4294967296\n",
        b"O \xff 2 3\n",
    )
    for line in malformed:
        assert parse_encoder_line(line) is None


def test_rejects_integer_overflow():
    too_large = str(2**63).encode()
    assert parse_encoder_line(b"O " + too_large + b" 0 0\n") is None


def test_format_velocity_command():
    assert format_velocity_command(0.08, 0.0) == "V 0.080000 0.000000\n"


def test_format_velocity_command_rejects_non_finite():
    try:
        format_velocity_command(math.nan, 0.0)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_format_config_command():
    assert format_config_command("min_duty", 60.0) == "C min_duty 60\n"
    assert format_config_command("closed_loop", 1.0) == "C closed_loop 1\n"
    assert format_config_command("kp", 250.5) == "C kp 250.5\n"


def test_format_config_command_wheel_geometry():
    mpt = 2 * math.pi * 0.03340 / 2481.9
    line = format_config_command("mpt_l", mpt)
    assert line.startswith("C mpt_l 8.4555") and line.endswith("e-05\n")
    assert float(line.split()[2]) == pytest.approx(mpt, rel=1e-8)
    assert format_config_command("sep", 0.395) == "C sep 0.395\n"


def test_wheel_geometry_commands():
    lines = wheel_geometry_commands(2481.9, 2468.6, 0.03340, 0.03306, 0.395)
    assert [line.split()[1] for line in lines] == ["mpt_l", "mpt_r", "sep"]
    assert float(lines[1].split()[2]) == pytest.approx(2 * math.pi * 0.03306 / 2468.6, rel=1e-8)
    # Not configured (0) -> nothing sent, the firmware keeps its default.
    assert wheel_geometry_commands(0.0, 0.0, 0.0, 0.0, 0.0) == []
    assert [line.split()[1] for line in wheel_geometry_commands(2481.9, 0.0, 0.0334, 0.033, 0.0)] == ["mpt_l"]


@pytest.mark.parametrize("name, value", [("speed", 1.0), ("kp", -1.0), ("ki", math.inf)])
def test_format_config_command_rejects_unknown_or_invalid(name, value):
    with pytest.raises(ValueError):
        format_config_command(name, value)


def test_parse_config_ack():
    assert parse_config_ack(b"K min_duty 60\n") == ("min_duty", "60")
    assert parse_config_ack("K ERR speed\r\n") == ("ERR", "speed")
    assert parse_config_ack(b"O 1 2 3\n") is None
    assert parse_config_ack(b"K only\n") is None


def test_encoder_parser_ignores_config_acks():
    assert parse_encoder_line(b"K kp 250.000\n") is None


def test_parse_valid_battery_line():
    assert parse_battery_line(b"B 2280 32 120345\n") == (2280, 32, 120345)
    assert parse_battery_line("  B 0 1 0 \r\n") == (0, 1, 0)


def test_rejects_malformed_battery_lines():
    malformed = (
        b"",
        b"B 2280 32\n",
        b"B 2280 32 1 2\n",
        b"B -1 32 1\n",
        b"B 3301 32 1\n",
        b"B 2280 0 1\n",
        b"B 2280 65536 1\n",
        b"B 2280 32 4294967296\n",
        b"B x 32 1\n",
        b"O 1 2 3\n",
        b"B \xff 1 1\n",
    )
    for line in malformed:
        assert parse_battery_line(line) is None


def test_battery_line_is_not_an_encoder_line():
    assert parse_encoder_line(b"B 2280 32 120345\n") is None
