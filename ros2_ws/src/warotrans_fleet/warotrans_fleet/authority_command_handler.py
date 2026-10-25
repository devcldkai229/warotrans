"""EXTEND_AUTHORITY / RENEW_AUTHORITY / REVOKE_AUTHORITY MQTT handlers (Active Mode)."""

from __future__ import annotations

from warotrans_fleet.command_contract import (
    EXTEND_AUTHORITY,
    RENEW_AUTHORITY,
    REVOKE_AUTHORITY,
    CommandEnvelope,
)
from warotrans_fleet.command_router import AckDecision, CommandSession
from warotrans_fleet.traffic_client import TrafficClient, TrafficDecision


def _to_ack(decision: TrafficDecision) -> AckDecision:
    if decision.accepted:
        return AckDecision.accept(message=decision.message)
    return AckDecision.reject(reason_code=decision.reason_code or "AUTHORITY_REQUIRED", message=decision.message or "")


class ExtendAuthorityHandler:
    command_type = EXTEND_AUTHORITY

    def __init__(self, client: TrafficClient, *, expected_graph_hash: str | None = None) -> None:
        self._client = client
        self._expected_graph_hash = expected_graph_hash

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        del session
        return _to_ack(self._client.extend_authority(command, expected_graph_hash=self._expected_graph_hash))


class RenewAuthorityHandler:
    command_type = RENEW_AUTHORITY

    def __init__(self, client: TrafficClient) -> None:
        self._client = client

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        del session
        return _to_ack(self._client.renew_authority(command))


class RevokeAuthorityHandler:
    command_type = REVOKE_AUTHORITY

    def __init__(self, client: TrafficClient) -> None:
        self._client = client

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        del session
        return _to_ack(self._client.revoke_authority(command))
