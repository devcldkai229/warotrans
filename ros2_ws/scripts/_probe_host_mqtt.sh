#!/usr/bin/env bash
set -euo pipefail
echo WSL_OK
head -3 /etc/resolv.conf
HOST_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}' | head -1)
echo "HOST_IP=$HOST_IP"
timeout 2 bash -c "echo >/dev/tcp/127.0.0.1/1883" && echo MQTT_127_OK || echo MQTT_127_FAIL
timeout 2 bash -c "echo >/dev/tcp/${HOST_IP}/1883" && echo MQTT_HOST_OK || echo MQTT_HOST_FAIL
curl -fsS --max-time 3 "http://${HOST_IP}:5090/health" && echo || \
  curl -fsS --max-time 3 "http://127.0.0.1:5090/health" && echo || echo API_FAIL
# paho import check
python3 - <<'PY'
import sys
sys.path.insert(0, "/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_fleet")
try:
    from warotrans_fleet._vendor_path import ensure_vendor
    ensure_vendor()
except Exception as e:
    print("vendor_path_err", e)
try:
    import paho.mqtt.client as c
    print("PAHO_OK", getattr(c, "__file__", "?"))
except Exception as e:
    print("PAHO_FAIL", e)
PY
