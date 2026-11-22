"""MQTT over TLS with the robot's X.509 certificate (AWS IoT Core)."""

from __future__ import annotations

import ssl

import pytest

from warotrans_fleet.mqtt_publisher import MqttTlsConfig, with_tls


class _FakeClient:
    def __init__(self, client_id: str) -> None:
        self.client_id = client_id
        self.tls: dict | None = None

    def tls_set(self, **kwargs) -> None:
        self.tls = kwargs


def _files(tmp_path):
    cert = tmp_path / "RBT-001.cert.pem"
    key = tmp_path / "RBT-001.private.key"
    ca = tmp_path / "AmazonRootCA1.pem"
    for f in (cert, key, ca):
        f.write_text("x")
    return str(cert), str(key), str(ca)


def test_every_new_client_presents_the_thing_certificate_over_tls12_or_later(tmp_path):
    cert, key, ca = _files(tmp_path)
    factory = with_tls(_FakeClient, MqttTlsConfig(cert_file=cert, key_file=key, ca_file=ca))

    client = factory("RBT-001")

    assert client.client_id == "RBT-001"
    assert client.tls == {
        "ca_certs": ca,
        "certfile": cert,
        "keyfile": key,
        "cert_reqs": ssl.CERT_REQUIRED,
        "tls_version": ssl.PROTOCOL_TLS_CLIENT,
    }


def test_without_a_ca_file_the_system_trust_store_is_used(tmp_path):
    cert, key, _ = _files(tmp_path)

    client = with_tls(_FakeClient, MqttTlsConfig(cert_file=cert, key_file=key))("RBT-002")

    assert client.tls["ca_certs"] is None


@pytest.mark.parametrize("missing", ["cert", "key", "ca"])
def test_a_missing_certificate_file_stops_the_start_instead_of_retrying_forever(tmp_path, missing):
    cert, key, ca = _files(tmp_path)
    config = {
        "cert": MqttTlsConfig(cert_file=str(tmp_path / "nope.pem"), key_file=key, ca_file=ca),
        "key": MqttTlsConfig(cert_file=cert, key_file="", ca_file=ca),
        "ca": MqttTlsConfig(cert_file=cert, key_file=key, ca_file=str(tmp_path / "nope-ca.pem")),
    }[missing]

    with pytest.raises(ValueError):
        with_tls(_FakeClient, config)
