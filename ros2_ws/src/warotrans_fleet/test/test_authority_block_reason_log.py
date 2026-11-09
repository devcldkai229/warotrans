"""The reason the gate reports is the one that really blocks the robot: it is all a log has to explain a stop."""

from __future__ import annotations

from warotrans_fleet.semantic_traffic_client import SemanticTrafficClient


def _client() -> tuple[SemanticTrafficClient, list[bool]]:
    gate: list[bool] = []
    client = SemanticTrafficClient(
        robot_code="RBT-TEST",
        topic_prefix="warotrans/v1",
        fingerprint="fp",
        publish_json=lambda *_a, **_k: None,
        set_authority_ok=gate.append,
        zones=[],
    )
    return client, gate


def test_no_grant_closes_the_gate_and_says_so():
    client, gate = _client()
    assert client.tick((0.0, 0.0, 0.0)) is False
    assert gate[-1] is False
    assert client.last_block_reason == "NO_GRANT"


def test_a_plan_rejection_is_what_the_gate_reports():
    client, _ = _client()
    client.on_plan_reject("KEEPOUT_VIOLATION", waiting="None")
    client.tick((0.0, 0.0, 0.0))
    assert client.last_block_reason.startswith("KEEPOUT_VIOLATION")


def test_a_revoke_is_what_the_gate_reports():
    client, _ = _client()
    client.on_revoke()
    client.tick((0.0, 0.0, 0.0))
    assert client.last_block_reason == "REVOKED"
