from warotrans_hardware.serial_protocol import format_velocity_command, parse_encoder_line


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
    import math

    try:
        format_velocity_command(math.nan, 0.0)
        assert False, "expected ValueError"
    except ValueError:
        pass
