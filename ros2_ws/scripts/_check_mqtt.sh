#!/usr/bin/env bash
HOST_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}' | head -1)
echo "HOST_IP=$HOST_IP"
for h in 127.0.0.1 "$HOST_IP" localhost; do
  if timeout 2 bash -c "echo >/dev/tcp/$h/1883" 2>/dev/null; then
    echo "MQTT_OK $h:1883"
  else
    echo "MQTT_FAIL $h:1883"
  fi
done
if command -v mosquitto_pub >/dev/null; then
  mosquitto_pub -h 127.0.0.1 -t warotrans/v1/ping -m hello -q 0 && echo PUB_127_OK || echo PUB_127_FAIL
  mosquitto_pub -h "$HOST_IP" -t warotrans/v1/ping -m hello -q 0 && echo PUB_HOST_OK || echo PUB_HOST_FAIL
fi
