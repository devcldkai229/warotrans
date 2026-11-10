"""Real robot + authority gate: the gate forwards to cmd_vel_motor and trusts only robot_gateway (launch wiring)."""

import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # noqa: E402

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "navigation.launch.py"


def _gate_node(args: dict[str, str]):
    spec = importlib.util.spec_from_file_location("nav_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            ctx.launch_configurations[entity.name] = args.get(entity.name, entity.default_value[0].text if entity.default_value else "")
    setup = next(e for e in ld.entities if isinstance(e, OpaqueFunction))
    actions = setup.execute(ctx) or []
    return [a for a in actions if getattr(a, "node_executable", None) == "cmd_vel_authority_gate"
            or getattr(a, "_Node__node_executable", None) == "cmd_vel_authority_gate"]


def _text(value):
    if isinstance(value, (tuple, list)):
        # launch_ros keeps string values as a YAML document: read it back like the node does.
        import yaml

        return yaml.safe_load("".join(getattr(t, "text", str(t)) for t in value))
    return value


def _params(node) -> dict:
    out = {}
    for p in node._Node__parameters:  # noqa: SLF001 — launch_ros keeps them private
        if isinstance(p, dict):
            for k, v in p.items():
                out[_text(k)] = _text(v)
    return out


def test_gate_on_real_robot_forwards_to_motor_topic_and_trusts_only_the_gateway():
    gates = _gate_node({"enable_authority_gate": "true", "enable_cmd_vel_pipeline": "true",
                        "gate_output_topic": "cmd_vel_motor", "required_authority_publisher": "robot_gateway"})
    assert len(gates) == 1
    params = _params(gates[0])
    assert params["output_topic"] == "cmd_vel_motor"
    assert params["required_authority_publisher"] == "robot_gateway"


def test_gate_off_keeps_legacy_pipeline():
    assert _gate_node({"enable_authority_gate": "false"}) == []
