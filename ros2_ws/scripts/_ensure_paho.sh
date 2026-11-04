#!/usr/bin/env bash
set -e
python3 - <<'PY'
try:
    import paho.mqtt.client as c
    print("PAHO_ALREADY", getattr(c, "__file__", "?"))
except Exception as e:
    print("PAHO_MISSING", e)
    raise SystemExit(2)
PY
exit 0
