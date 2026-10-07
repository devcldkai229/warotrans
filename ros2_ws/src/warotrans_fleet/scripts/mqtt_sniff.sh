#!/usr/bin/env bash
# Subscribe to WaroTrans robot MQTT topics for developer verification.
# Requires: mosquitto_sub (mosquitto-clients)
#
# Usage:
#   ./mqtt_sniff.sh [host] [port] [robotCode]
# Example:
#   ./mqtt_sniff.sh localhost 1883 RBT-001

set -euo pipefail

HOST="${1:-localhost}"
PORT="${2:-1883}"
ROBOT="${3:-}"

if [[ -n "${ROBOT}" ]]; then
  TOPIC="warotrans/v1/robots/${ROBOT}/#"
else
  TOPIC="warotrans/v1/robots/+/#"
fi

echo "Listening on ${HOST}:${PORT} topic ${TOPIC}"
echo "Expect ~1 Hz heartbeat and ~5 Hz telemetry JSON."
exec mosquitto_sub -h "${HOST}" -p "${PORT}" -t "${TOPIC}" -v
