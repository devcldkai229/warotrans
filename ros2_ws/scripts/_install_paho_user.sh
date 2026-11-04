#!/usr/bin/env bash
# Install paho-mqtt into user site without sudo.
set -uo pipefail

if python3 - <<'PY'
import importlib.util, sys
ok = importlib.util.find_spec("paho") is not None
print("PAHO_PRESENT" if ok else "PAHO_MISSING")
sys.exit(0 if ok else 2)
PY
then
  python3 - <<'PY'
import paho.mqtt.client as c
print("PAHO_ALREADY", c.__file__)
PY
  exit 0
fi

set -e
echo "Installing paho-mqtt via ensurepip/pip --user ..."
python3 -m ensurepip --upgrade --user || true
python3 -m pip install --user --upgrade pip || true
python3 -m pip install --user paho-mqtt

python3 - <<'PY'
import paho.mqtt.client as c
print("PAHO_OK", c.__file__)
PY
