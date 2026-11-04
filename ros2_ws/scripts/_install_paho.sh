#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
if ! python3 - <<'PY'
import importlib.util
print("has_paho", importlib.util.find_spec("paho") is not None)
PY
then
  true
fi
if dpkg -s python3-paho-mqtt >/dev/null 2>&1; then
  echo "apt package python3-paho-mqtt already installed"
else
  sudo apt-get update -qq
  sudo apt-get install -y python3-paho-mqtt python3-pip || sudo apt-get install -y python3-pip
fi
if ! python3 - <<'PY'
import paho.mqtt.client as c
print("PAHO_OK", getattr(c, "__file__", "?"))
PY
then
  python3 -m ensurepip --user || true
  python3 -m pip install --user paho-mqtt
  python3 - <<'PY'
import paho.mqtt.client as c
print("PAHO_OK_PIP", getattr(c, "__file__", "?"))
PY
fi
