import math

import pytest

from warotrans_hardware.serial_protocol import (
    format_config_command,
    format_velocity_command,
    parse_config_ack,
    parse_encoder_line,
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
