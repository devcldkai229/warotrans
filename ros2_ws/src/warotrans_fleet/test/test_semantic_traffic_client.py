"""Unit tests for Stage-3 SemanticTrafficClient (no ROS)."""

from __future__ import annotations

from warotrans_fleet import semantic_traffic_client as stc
from warotrans_fleet.semantic_traffic_client import SemanticTrafficClient

PATH_A = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]


def _granted_client(ok_flags, *, now=None, seq=1):
    kwargs = {} if now is None else {"now": now}
    client = SemanticTrafficClient(
        robot_code="RBT-001",
        topic_prefix="warotrans/v1",
        fingerprint="fp1",
        publish_json=lambda *_: None,
        set_authority_ok=lambda v: ok_flags.append(v),
        **kwargs,
    )
    client.on_plan(PATH_A)
    assert client.on_authority_payload(_grant(seq=seq)) is True
    return client


def _grant(*, seq=1, rev=1, epoch=1, authority_id="a1", expires_at=None):
    body = {
        "authorityId": authority_id,
        "epoch": epoch,
        "seq": seq,
        "semanticMapFingerprint": "fp1",
        "planRevision": rev,
        "resourceIds": ["cell:0:0"],
        "pathHash": "h",
    }
    if expires_at is not None:
        body["expiresAt"] = expires_at
    return body


def test_revoke_closes_the_gate_immediately():
    ok_flags = []
    client = _granted_client(ok_flags)

    client.on_revoke()
    client.tick()

    # Was: open for replan_grace_sec (3 s) after REVOKE. The backend only revokes from a quarantine (STOP_NOW).
    assert ok_flags[-2:] == [False, False]
    assert client.grant is None


def test_grant_expired_on_arrival_never_opens_the_gate():
    ok_flags = []
    client = SemanticTrafficClient(
        robot_code="RBT-001", topic_prefix="warotrans/v1", fingerprint="fp1",
        publish_json=lambda *_: None, set_authority_ok=lambda v: ok_flags.append(v),
        now=lambda: 2_000_000_000.0,
    )
    client.on_plan(PATH_A)

    assert client.on_authority_payload(_grant(expires_at="2000-01-01T00:00:00Z")) is False
    assert True not in ok_flags


def test_grant_without_plan_revision_is_refused():
    ok_flags = []
    client = SemanticTrafficClient(
        robot_code="RBT-001", topic_prefix="warotrans/v1", fingerprint="fp1",
        publish_json=lambda *_: None, set_authority_ok=lambda v: ok_flags.append(v),
    )
    client.on_plan(PATH_A)

    assert client.on_authority_payload(_grant(rev=0)) is False
    assert True not in ok_flags


def test_redelivered_or_older_grant_does_not_roll_the_grant_back():
    ok_flags = []
    client = _granted_client(ok_flags, seq=5)

    assert client.on_authority_payload(_grant(seq=5)) is False      # QoS1 duplicate
    assert client.on_authority_payload(_grant(seq=3)) is False      # older
    assert client.on_authority_payload(_grant(seq=9, epoch=0, authority_id="old")) is False
    assert client.grant.seq == 5
    assert client.on_authority_payload(_grant(seq=6)) is True


def test_old_grant_bridges_a_replan_only_for_the_grace_window(monkeypatch):
    mono = {"t": 100.0}
    monkeypatch.setattr(stc.time, "monotonic", lambda: mono["t"])
    ok_flags = []
    client = _granted_client(ok_flags)
    client.replan_grace_sec = 3.0

    # Robot replans (new path), backend has not granted revision 2 yet: old grant bridges the gap.
    client.on_plan([(0.0, 0.0, 0.0), (0.0, 1.0, 1.57)])
    client.tick()
    assert ok_flags[-1] is True

    # Keeps replanning while the backend refuses: later revisions must not extend the window.
    for i in range(1, 6):
        mono["t"] += 1.0
        client.on_plan([(0.1 * i, 0.0, 0.0), (0.0, 2.0 + i, 1.57)])
        client.tick()
    # Was: the old 30 s grant stayed valid for every new path the backend never granted.
    assert ok_flags[-1] is False
    assert client.is_grant_valid() is False


def test_grant_for_the_new_revision_ends_the_handoff(monkeypatch):
    mono = {"t": 100.0}
    monkeypatch.setattr(stc.time, "monotonic", lambda: mono["t"])
    ok_flags = []
    client = _granted_client(ok_flags)

    client.on_plan([(0.0, 0.0, 0.0), (0.0, 1.0, 1.57)])
    assert client.on_authority_payload(_grant(seq=2, rev=2)) is True
    mono["t"] += 10.0
    client.tick()
    assert ok_flags[-1] is True


def test_plan_propose_closes_gate_until_grant():
    published = []
    ok_flags = []

    client = SemanticTrafficClient(
        robot_code="RBT-SIM-01",
        topic_prefix="warotrans/v1",
        fingerprint="fp1",
        publish_json=lambda t, b: published.append((t, b)),
        set_authority_ok=lambda v: ok_flags.append(v),
    )
    client.on_plan([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], goal=(1.0, 0.0, 0.0))
    assert ok_flags[-1] is False
    assert published and published[-1][1]["planRevision"] == 1
    assert published[-1][0].endswith("/traffic_plan")

    client.on_authority_payload(
        {
            "authorityId": "a1",
            "epoch": 1,
            "seq": 1,
            "semanticMapFingerprint": "fp1",
            "planRevision": 1,
            "resourceIds": ["zone:Z1", "cell:0:0"],
            "pathHash": "abc",
        }
    )
    assert ok_flags[-1] is True
    assert client.grant is not None
    assert "zone:Z1" in client.held_resources


def test_stale_plan_revision_rejected():
    ok_flags = []
    client = SemanticTrafficClient(
        robot_code="RBT-X",
        topic_prefix="warotrans/v1",
        fingerprint="fp",
        publish_json=lambda *_: None,
        set_authority_ok=lambda v: ok_flags.append(v),
    )
    client.plan_revision = 2
    assert client.on_authority_payload(
        {
            "authorityId": "a",
            "epoch": 1,
            "seq": 1,
            "semanticMapFingerprint": "fp",
            "planRevision": 1,
            "resourceIds": [],
            "pathHash": "",
        }
    ) is False
    # The gate starts closed and a stale grant publishes nothing (ok_flags[-1] raised on the empty list).
    assert True not in ok_flags
    assert client.grant is None


def test_identical_plan_debounced_does_not_bump_revision():
    published = []
    ok_flags = []
    client = SemanticTrafficClient(
        robot_code="RBT-SIM-01",
        topic_prefix="warotrans/v1",
        fingerprint="fp1",
        publish_json=lambda t, b: published.append((t, b)),
        set_authority_ok=lambda v: ok_flags.append(v),
    )
    poses = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    client.on_plan(poses)
    client.on_authority_payload(
        {
            "authorityId": "a1",
            "epoch": 1,
            "seq": 1,
            "semanticMapFingerprint": "fp1",
            "planRevision": 1,
            "resourceIds": ["cell:0:0"],
            "pathHash": "h",
        }
    )
    n = len(published)
    rev = client.plan_revision
    client.on_plan(poses)  # identical path
    assert client.plan_revision == rev
    assert len(published) == n
    assert ok_flags[-1] is True


def test_tick_republishes_authority_ok_while_grant_valid():
    ok_flags = []
    now = {"t": 1000.0}
    client = SemanticTrafficClient(
        robot_code="RBT-SIM-01",
        topic_prefix="warotrans/v1",
        fingerprint="fp1",
        publish_json=lambda *_: None,
        set_authority_ok=lambda v: ok_flags.append(v),
        now=lambda: now["t"],
        authority_loss_timeout_sec=3.0,
    )
    client.on_plan([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    client.on_authority_payload(
        {
            "authorityId": "a1",
            "epoch": 1,
            "seq": 2,
            "semanticMapFingerprint": "fp1",
            "planRevision": 1,
            "resourceIds": [],
            "pathHash": "",
            "expiresAt": "2000-01-01T00:00:03Z",
        }
    )
    # Force known expiry relative to now
    client.grant.expires_at = now["t"] + 3.0
    ok_flags.clear()
    client.tick()
    client.tick()
    assert ok_flags == [True, True]
    now["t"] += 4.0
    client.tick()
    assert ok_flags[-1] is False


def test_publish_cleared_resources_emits_progress():
    published = []
    client = SemanticTrafficClient(
        robot_code="RBT-SIM-01",
        topic_prefix="warotrans/v1",
        fingerprint="fp1",
        publish_json=lambda t, b: published.append((t, b)),
        set_authority_ok=lambda *_: None,
    )
    client.on_plan([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    client.on_authority_payload(
        {
            "authorityId": "a1",
            "epoch": 5,
            "seq": 7,
            "semanticMapFingerprint": "fp1",
            "planRevision": 1,
            "resourceIds": ["cell:0:0", "cell:1:0"],
            "pathHash": "",
        }
    )
    client.publish_cleared_resources(["cell:0:0"], x=1.0, y=0.0, yaw=0.0, localization_ok=True)
    assert published[-1][0].endswith("/traffic_progress")
    body = published[-1][1]
    assert body["event"] == "RESOURCES_CLEARED"
    assert body["clearedResourceIds"] == ["cell:0:0"]
    assert body["epoch"] == 5
    assert body["seq"] == 8
